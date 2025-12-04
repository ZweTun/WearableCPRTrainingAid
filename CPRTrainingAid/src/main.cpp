#include <Arduino.h>
#include <Wire.h>
#include <iostream>
#include <cstring>

#include "CPRTrainingModel.h"

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"

// ---------- TensorFlow Lite Micro  ----------
constexpr int kTensorArenaSize = 40 * 1024;
alignas(16) uint8_t tensor_arena[kTensorArenaSize];

const tflite::Model*         model       = nullptr;
tflite::MicroInterpreter*    interpreter = nullptr;

// Anomaly thresholds (reconstruction error outside this range => anomaly)
static const float lower_threshold = 0.00755;
static const float upper_threshold = 0.00928;


// ---------- Vibration Motor Config ----------
#define VIBRATION_MOTOR_PIN 9


// ---------- I2C pins & ICM20600 config ----------
#define I2C_SDA D2
#define I2C_SCL D3
#define ICM20600_ADDR 0x69

#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_CONFIG 0x1C
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_XOUT_H 0x3B
#define REG_WHO_AM_I     0x75

// conversions for ±2g accel, ±250 dps gyro
static const float ACC_LSB_PER_G   = 16384.0f;
static const float GYR_LSB_PER_DPS = 131.0f;

// time window 
static const int WINDOW_SIZE = 300; // timesteps

// Per-channel normalization from training data
static const float training_min=  -250.137405;
static const float training_max= 248.175568;

// Vibration motor modes 
// 0=CPR too slow, 1=good, 2=CPR too fast
// 0 is burst
// 1 is off
// 2 is steady on 
volatile int motorMode = 1; 

// ---------- I2C helpers ----------
static bool i2cWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission((uint8_t)ICM20600_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool i2cReadBytes(uint8_t reg, uint8_t* buf, size_t len) {
  Wire.beginTransmission((uint8_t)ICM20600_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((uint8_t)ICM20600_ADDR, (uint8_t)len) != (int)len) return false;
  for (size_t i = 0; i < len; ++i) {
    buf[i] = Wire.read();
  }
  return true;
}

static bool icmInit(uint8_t& who) {
  who = 0;
  if (!i2cWrite(REG_PWR_MGMT_1, 0x01)) return false;  // wake, PLL
  delay(50);
  if (!i2cWrite(REG_ACCEL_CONFIG, 0x00)) return false; // ±2g
  if (!i2cWrite(REG_GYRO_CONFIG,  0x00)) return false; // ±250 dps
  delay(10);
  return i2cReadBytes(REG_WHO_AM_I, &who, 1);
}

static bool icmRead(float& ax,float& ay,float& az,
                    float& gx,float& gy,float& gz) {
  uint8_t raw[14];
  if (!i2cReadBytes(REG_ACCEL_XOUT_H, raw, sizeof(raw))) return false;

  auto s16 = [&](int i)->int16_t {
    return (int16_t)((raw[i] << 8) | raw[i+1]);
  };

  ax = s16(0)  / ACC_LSB_PER_G;
  ay = s16(2)  / ACC_LSB_PER_G;
  az = s16(4)  / ACC_LSB_PER_G;

  gx = s16(8)  / GYR_LSB_PER_DPS;
  gy = s16(10) / GYR_LSB_PER_DPS;
  gz = s16(12) / GYR_LSB_PER_DPS;

  return true;
}


hw_timer_t* timer = NULL;


// According to motorMode, set vibration motor output
void IRAM_ATTR onTimer() {


  // Serial.print("Timer Interrupt! motorMode=");
  // Serial.print(motorMode);
  // Serial.print("\n");

}


// Setup hardware timer to trigger every 100 ms (1000 Hz)
void setupTimer() {
    int timer_divider = 80;        
    int timer_interval_us = 100000;   

    timer = timerBegin(0, timer_divider, true);  // Timer 0, count up
    timerAttachInterrupt(timer, &onTimer, true); // Attach ISR

    // Set alarm to fire every 10,000 µs
    timerAlarmWrite(timer, timer_interval_us, true);
    timerAlarmEnable(timer);
}

void setup() {
  // ESP32S3 XIAO USB CDC setup
  Serial.begin(115200);
  
  // Wait for USB CDC to be ready on ESP32S3 in Windows environment
  while (!Serial && millis() < 5000) {
    delay(10);
  }
  delay(1000);
  Serial.flush();
  Serial.print("========================================\r\n");



  // I2C + IMU
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  Serial.print("[I2C] Initializing ICM20600...\r\n");
  
  uint8_t who = 0;
  if (icmInit(who)) {
    Serial.print("[I2C] ICM20600 OK, WHO_AM_I=0x");
    Serial.print(who, HEX);
    Serial.print(" (addr 0x");
    Serial.print(ICM20600_ADDR, HEX);
    Serial.print(")\r\n");
  } else {
    Serial.print("[I2C] ICM init FAILED\r\n");
  }


  // TensorFlow Lite Micro setup
    model = tflite::GetModel(cpr_training_model);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("ERROR: Model version mismatch!");
    return;
  }

  // All op resolvers needed should be added here
  static tflite::AllOpsResolver micro_op_resolver;
  static tflite::MicroInterpreter static_interpreter(
      model, micro_op_resolver, tensor_arena, kTensorArenaSize);


  interpreter = &static_interpreter;

  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("ERROR: Tensor alloc failed!");
    return;
  }



  Serial.println("Model ready");

  // Print input/output tensor sizes for debugging
  {
    TfLiteTensor* in = interpreter->input(0);
    int input_elements = in->bytes / sizeof(float);
    Serial.print("Input elements: "); Serial.println(input_elements);
    TfLiteTensor* out = interpreter->output(0);
    int output_elements = out->bytes / sizeof(float);
    Serial.print("Output elements: "); Serial.println(output_elements);
  }



  // Vibration motor pin
  pinMode(VIBRATION_MOTOR_PIN, OUTPUT);
  digitalWrite(VIBRATION_MOTOR_PIN, LOW);


  // Setup hardware timer 
  //setupTimer();
  
  Serial.print("========================================\r\n");
  Serial.print("[SETUP] Ready!\r\n");
  Serial.print("========================================\r\n");


}





void guideCPR(float recon_error) {
  // Simple vibration motor feedback based on reconstruction error
  if (recon_error < lower_threshold) {
    // Too low movement - steady vibration
    motorMode = 0;
  } else if (recon_error > upper_threshold) {
    // Too high movement - no vibration
    motorMode = 2;
  } else {
    // Good movement - short pulse
    motorMode = 1;
  }


  switch (motorMode) {
    case 0: // CPR too slow - burst
      // digitalWrite(VIBRATION_MOTOR_PIN, HIGH);
      // delay(200);
      // digitalWrite(VIBRATION_MOTOR_PIN, LOW);
      
      break;
    case 1: // good - off
      digitalWrite(VIBRATION_MOTOR_PIN, LOW);
      break;
    case 2: // CPR too fast - steady on
      digitalWrite(VIBRATION_MOTOR_PIN, HIGH);
      break;
    default:
      digitalWrite(VIBRATION_MOTOR_PIN, LOW);
      break;
  }

}

// Retutns reconstrution error for auto encoder model
float performInference(float* input_data) {
  if (!interpreter) {
    Serial.println("ERROR: interpreter null");
    return -1.0f;
  }
  TfLiteTensor* input = interpreter->input(0);
  int input_elements = input->bytes / sizeof(float);


  // Copy into interpreter input
  for (int i = 0; i < input_elements; ++i) {
    input->data.f[i] = input_data[i];
  }


  Serial.println();

  Serial.println("About to Invoke");
  if (interpreter->Invoke() != kTfLiteOk) {
    Serial.println("ERROR: Inference failed!");
    return -1.0f;
  }
  Serial.println("Invoke OK");

  TfLiteTensor* output = interpreter->output(0);
  int out_elems = output->bytes / sizeof(float);
  // Serial.print("OUT elems:"); Serial.println(out_elems);

  
  // Compute per timestep MAE averaged over features, then average over timesteps
  int timesteps = input_elements / 6;
  float sum_over_timesteps = 0.0f;
  for (int t = 0; t < timesteps; ++t) {
    float sum_features = 0.0f;
    for (int f = 0; f < 6; ++f) {
      int idx = t * 6 + f;
      float diff = output->data.f[idx] - input_data[idx];
      if (diff < 0.0f) diff = -diff;
      sum_features += diff;
    }

    // average over features
    float mae_t = sum_features / 6.0f;
    sum_over_timesteps += mae_t;
  }
  // average over timesteps
  float recon_error = sum_over_timesteps / (float)timesteps;
 // Serial.print("Computed MAE recon_error:"); Serial.println(recon_error, 6);
  bool is_anomaly = (recon_error < lower_threshold) || (recon_error > upper_threshold);
  Serial.print("ANOMALY:"); Serial.println(is_anomaly ? "TRUE" : "FALSE");
  return recon_error;
 
}



// Normalize one IMU sample (6 floats)
void normalizeIMU(const float vals[6], float norm_vals[6]) {
    
    // Accelerometer channels 0,1,2
    for (int i = 0; i < 3; i++) {
        float denom = acc_max[i] - acc_min[i];
        if (denom < 1e-6f) denom = 1e-6f;   // prevent divide-by-zero
        float x = (vals[i] - acc_min[i]) / denom;

        // clamp
        if (x < 0.0f) x = 0.0f;
        if (x > 1.0f) x = 1.0f;
        norm_vals[i] = x;
    }

    // Gyroscope channels 3,4,5
    for (int i = 0; i < 3; i++) {
        float denom = gyro_max[i] - gyro_min[i];
        if (denom < 1e-6f) denom = 1e-6f;
        float x = (vals[i + 3] - gyro_min[i]) / denom;

        // clamp
        if (x < 0.0f) x = 0.0f;
        if (x > 1.0f) x = 1.0f;
        norm_vals[i + 3] = x;
    }
}




void loop() {
  static uint32_t last = 0;


  float ax, ay, az, gx, gy, gz;
  if (!icmRead(ax, ay, az, gx, gy, gz)) {
    return;
  }



  //9 float32 little-endian payload: ax..gz + mx,my,mz (mag set to 0)
  float mx = 0, my = 0, mz = 0;
  float vals[6] = { ax, ay, az, gx, gy, gz };

  // Normalize per-channel using training min/max 
  float norm_vals[6];

  normalizeIMU(vals, norm_vals);
  // for (int c = 0; c < 6; ++c) {
  //   float denom = training_max - training_min;
  //   norm_vals[c] = (vals[c] - training_min) / denom;
  //   // Clamp to [0,1] 
  //   if (norm_vals[c] < 0.0f) norm_vals[c] = 0.0f;
  //   if (norm_vals[c] > 1.0f) norm_vals[c] = 1.0f;
  // }

   

  // Copy sensor data to model input buffer
  static float input_buffer[6 * WINDOW_SIZE];

  // Flush buffer after WINDOW_SIZE samples and perform inference
  static int count = 0;
  if (count >= WINDOW_SIZE) {
    // Perform inference on the full buffer
    float recon_error = performInference(input_buffer);
    Serial.print("RECON_ERROR:");
    Serial.println(recon_error, 6); // 6 decimal places
    guideCPR(recon_error);
    // reset count and zero the buffer
    for (int i = 0; i < 6 * WINDOW_SIZE; ++i) {
      input_buffer[i] = 0.0f;
    }
    count = 0;
  }

  int offset = count * 6;
  for (int i = 0; i < 6; ++i) {
    input_buffer[offset + i] = norm_vals[i];
  }

  count++; 

  // Serial.print("DATA:");
  // for (int i = 0; i < 6; ++i) {
  //   if (i) Serial.print(',');
  //   Serial.print(vals[i], 6); // 6 decimal places
  // }
  // Serial.println();




  // 100hz 
  delay(10);

}
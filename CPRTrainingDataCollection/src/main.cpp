#include <Arduino.h>
#include <Wire.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

// ---------- Vibration Motor config ----------
#define VIBRATION_PIN 9
const unsigned long BEAT_PERIOD_MS = 60000UL / 140;  //  bpm control
const unsigned long PULSE_WIDTH_MS = 80;             // how long the buzz lasts

unsigned long lastBeatStart = 0;


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



void setup() {
  // ESP32S3 XIAO USB CDC setup
  Serial.begin(115200);
  
  // Wait for USB CDC to be ready on ESP32S3 in Windows environment
  while (!Serial && millis() < 5000) {
    delay(10);
  }
  delay(1000);
  Serial.flush();
  
  Serial.println("\n[BOOT] HTML-compatible Wand streamer");
  


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


  // Vibration motor
  pinMode(VIBRATION_PIN, OUTPUT);
  digitalWrite(VIBRATION_PIN, LOW); // off
  
  Serial.print("========================================\r\n");
  Serial.print("[SETUP] Ready for Training!\r\n");
  Serial.print("========================================\r\n");


}

unsigned long lastPulseTime = 0;
int motor  = LOW;
void loop() {
  static uint32_t last = 0;
  unsigned long now = millis();
  
  // start a new beat when period has passed
  // if (now - lastBeatStart >= BEAT_PERIOD_MS) {
  //     lastBeatStart = now;
  // }

  // turn motor ON for part of the beat, OFF otherwise
  // if (now - lastBeatStart < PULSE_WIDTH_MS) {
  //     digitalWrite(VIBRATION_PIN, HIGH);
  // } else {
  //     digitalWrite(VIBRATION_PIN, LOW);
  // }

    // if (now - lastPulseTime >= 200) {  // faster burst rate
    //   lastPulseTime = now;
      
    //   digitalWrite(VIBRATION_PIN, motor);
    //   motor = !motor;
    // }
  //  digitalWrite(VIBRATION_PIN, HIGH);
  //     delay(200);
  digitalWrite(VIBRATION_PIN, HIGH);

  


  float ax, ay, az, gx, gy, gz;
  if (!icmRead(ax, ay, az, gx, gy, gz)) {
    return;
  }



  // // 9 float32 little-endian payload: ax..gz + mx,my,mz (mag set to 0)
  float mx = 0, my = 0, mz = 0;
  float vals[6] = { ax, ay, az, gx, gy, gz};
  Serial.print("DATA:");
  for (int i = 0; i < 6; ++i) {
    if (i) Serial.print(',');
    Serial.print(vals[i], 6); // 6 decimal places
  }
  Serial.println();


  //Write to CSV files for training data 
  // /data/
  //   good/
  //       session1.csv
  //       session2.csv
  //   bad/
  //       session1.csv
  //       session2.csv
  // After collecting data, use CSV files for training  model.

  // 100hz 
  delay(10);

}
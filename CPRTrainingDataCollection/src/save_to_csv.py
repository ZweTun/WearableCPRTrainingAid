import serial
import csv
import time

PORT = "COM3"  
BAUD = 115200

ser = serial.Serial(PORT, BAUD)
time.sleep(2)

filename = "data/good/v_good_session1.csv"

# filename = "data/bad/v_bad_session2.csv"
f = open(filename, "w", newline="")
writer = csv.writer(f)
writer.writerow(["ax","ay","az","gx","gy","gz"])

print("Recording CTRL+C to stop")


#data arrives at ~100Hz so 100 data points per second
while True:
    line = ser.readline().decode().strip()
    if line.startswith("DATA:"):
        values = line.replace("DATA:", "").split(",")
        writer.writerow(values)

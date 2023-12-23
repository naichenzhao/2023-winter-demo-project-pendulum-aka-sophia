#include <Arduino.h>
#include "MPU9250.h"
#include <MadgwickAHRS.h>
#include <RunningMedian.h>

Madgwick filter;
MPU9250 mpu;
RunningMedian samples = RunningMedian(5);

void print_roll_pitch_yaw();
void print_calibration();

// Set pins for I2C1
#define RX1 2
#define TX1 3

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX1, TX1);

  Wire.begin(19, 18);

  delay(500);
  if (!mpu.setup(0x69)) {
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(1000);
    }
  }
  filter.begin(25);
}


void loop() {
  mpu.update();
  samples.add(mpu.getRoll());

  // float ax, ay, az;
  // float gx, gy, gz;

  // // convert from raw data to gravity and degrees/second units
  // ax = mpu.getAccX();
  // ay = mpu.getAccY();
  // az = mpu.getAccZ();
  // gx = mpu.getGyroX();
  // gy = mpu.getGyroY();
  // gz = mpu.getGyroZ();

  // // update the filter, which computes orientation
  // filter.updateIMU(gx, gy, gz, ax, ay, az);

  // print the heading, pitch and roll
  int curr_angle = (int)(samples.getMedian() * 1000) - 1700;
  Serial.println(curr_angle);
  Serial1.println(curr_angle);
}




void print_roll_pitch_yaw()
{
  Serial.print("Yaw, Pitch, Roll: ");
  Serial.print(mpu.getYaw(), 2);
  Serial.print(", ");
  Serial.print(mpu.getPitch(), 2);
  Serial.print(", ");
  Serial.println(mpu.getRoll(), 2);
}

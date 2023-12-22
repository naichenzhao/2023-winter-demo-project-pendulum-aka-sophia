#include <Arduino.h>
#include "MPU9250.h"

MPU9250 mpu;

void print_roll_pitch_yaw();
void print_calibration();

// Set pins for I2C1
#define RX1 2
#define TX1 3

TwoWire I2C = TwoWire(1); // I2C2 bus

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

}


void loop() {
  mpu.update();
  int curr_angle = (int) (mpu.getRoll() * 1000) - 3200;
  // Serial.println(curr_angle);
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

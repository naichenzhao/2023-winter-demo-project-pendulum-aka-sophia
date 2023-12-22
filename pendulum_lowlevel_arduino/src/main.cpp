#include <Arduino.h>
#include <AccelStepper.h>
#include <SPI.h>

#define motorInterfaceType 1
AccelStepper step_motor = AccelStepper(motorInterfaceType, 5, 4);

void set_motor(int speed);

int sensor_val = 0;

// Set pins for I2C1
#define RX1 2
#define TX1 3

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX1, TX1);

  // step_motor.setCurrentPosition(0);
  // step_motor.setMaxSpeed(400000);
  // step_motor.setAcceleration(150000000);

  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
}

void loop() {
  float curr_angle = -((float)sensor_val)/1000;

  int getspeed = 100 * (int)curr_angle;

  if (curr_angle > 45 || curr_angle < -45) {
    set_motor(0);
  } else {
    set_motor(getspeed);
  }

  Serial.print(curr_angle);
  Serial.print("  ");
  Serial.println(getspeed);

  if (Serial1.available()) {
    String ser_read = Serial1.readStringUntil('\n');
    sensor_val = ser_read.toInt();
  }
}


void set_motor(int speed) {

  speed = (speed > 255)? 255:speed;
  speed = (speed < -255) ? -255 : speed;

  if (speed > 0) {
    analogWrite(5, speed);
    analogWrite(4, 0);
  } else {
    analogWrite(4, -speed);
    analogWrite(5, 0);
  }

}



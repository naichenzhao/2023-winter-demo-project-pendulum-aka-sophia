#include <Arduino.h>
#include <AccelStepper.h>
#include <RunningMedian.h>

#define motorInterfaceType 1
AccelStepper step_motor = AccelStepper(motorInterfaceType, 5, 4);

RunningMedian samples = RunningMedian(5);

void set_motor(int speed);

int sensor_val = 0;
float last_angle = 0;

// Set pins for I2C1
#define RX1 2
#define TX1 3

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX1, TX1);

  step_motor.setCurrentPosition(0);
  step_motor.setMaxSpeed(40000);
  step_motor.setAcceleration(200000);
}

int getspeed;

    void
    loop()
{
  float curr_angle = -((float)sensor_val) / 1000;

  // if (curr_angle < 5 && curr_angle > -5) {
  //   getspeed = (int)(1000 * curr_angle) + (6000 * (curr_angle - last_angle));
  // } else {
    getspeed = (int)(8000 * curr_angle) + (4000 * (curr_angle - last_angle));
  // }



  if (curr_angle > 45 || curr_angle < -45) {
    // samples.add(0);
    set_motor(0);
  } else {
    // samples.add(getspeed);
    set_motor(getspeed);
    }

  // set_motor(samples.getAverage());

  // Serial.println(curr_angle);
  // Serial.print("  ");
  // Serial.print(getspeed);
  // Serial.print("  ");
  // Serial.println(step_motor.speed());
  last_angle = curr_angle;
  if (Serial1.available()) {
    String ser_read = Serial1.readStringUntil('\n');
    sensor_val = ser_read.toInt();
  }
}

void set_motor(int speed)
{

  if (speed < 300 && speed > -300) {
    step_motor.stop();
  } else {
    step_motor.setSpeed(speed);
    step_motor.runSpeed();
  }
}

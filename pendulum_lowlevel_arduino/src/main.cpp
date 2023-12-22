#include <Arduino.h>
#include <AccelStepper.h>

#define motorInterfaceType 1
AccelStepper step_motor = AccelStepper(motorInterfaceType, 5, 4);

void set_motor(int speed);

int sensor_val = 0;

// Set pins for I2C1
#define RX1 2
#define TX1 3

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX1, TX1);

  step_motor.setCurrentPosition(0);
  step_motor.setMaxSpeed(400000);
  step_motor.setAcceleration(150000000);
}

void loop()
{
  float curr_angle = -((float)sensor_val) / 1000;

  int getspeed = 10000 * (int)curr_angle;

  if (curr_angle > 45 || curr_angle < -45)
  {
    set_motor(0);
  }
  else
  {
    set_motor(getspeed);
  }
  for (int i = 0; i < 8000; i++) {
    set_motor(20000);
  }

    for (int i = 0; i < 8000; i++) {
    set_motor(-20000);
  }

  Serial.print(curr_angle);
  Serial.print("  ");
  Serial.print(getspeed);
  Serial.print("  ");
  Serial.println(step_motor.speed());

  if (Serial1.available())
  {
    String ser_read = Serial1.readStringUntil('\n');
    sensor_val = ser_read.toInt();
  }
}

void set_motor(int speed)
{

  if (speed == 0)
  {
    step_motor.stop();
  }
  else
  {
    step_motor.setSpeed(speed);
    step_motor.runSpeed();
  }
}

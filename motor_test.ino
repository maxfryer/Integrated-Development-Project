#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
void setup() {
  // put your setup code here, to run once:
  AFMS.begin();

}

void loop() {
  // put your main code here, to run repeatedly:
  myMotor1->setSpeed(200);
  myMotor1->run(FORWARD);
  myMotor2->setSpeed(200);
  myMotor2->run(FORWARD);

}

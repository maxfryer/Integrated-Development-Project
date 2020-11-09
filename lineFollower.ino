//this is a test program that should contrain the robot to follow a line.

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);

int sensorPin = A0;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor


/*!
    @brief commands the motors to turn at the right speed to keep the robot on a line
    @param 1-3 sensor values, with sensor 1 being the middle. 
    @param If two given then left and right and if 3 then middle, left, right
*/
void followLine(int sensorVal1, int sensorVal2, int sensorVal3)
/*sensor1 needs to be the central slot.*/
{
    
}




void setup() {
    // put your setup code here, to run once:
    AFMS.begin();
    Serial.begin(9600); 
}

void loop() {
    // put your main code here, to run repeatedly:
    myMotor1->setSpeed(200);
    myMotor1->run(FORWARD);
    myMotor2->setSpeed(200);
    myMotor2->run(FORWARD);

    followLine(1,2,3);

}

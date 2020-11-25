//this is a test program that should contrain the robot to follow a line.

//THE LOOP AT THE BOTTOM RUNS THE WHOLE PROGRAM
//SET THE CONTROLLER GAINS AT THE START on the line Robot Bot(2,0.5,0.5)
//these can be changed, eg for a pure proportional, choose (2,0,0)
//(proportional, integral, derivative)
#include <Wire.h>

#include <Adafruit_MotorShield.h>

#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor * myMotorLeft = AFMS.getMotor(1);
Adafruit_DCMotor * myMotorRight = AFMS.getMotor(2);

bool run = false;


class Robot {
    public:
        /*ALL PINS */
        int frontLeft = A0; // input pin for FRONT LEFT light sensor
        int frontRight = A1;
        int offAxisRight = 3;
        int offAxisLeft = 4;
        int backMiddle = 5;
        int startButtonPin = 2;
        int distanceSensor = A2;
        

        int ledPinFirst = 13;
        int ledPinSecond = 12;
        int ledPinThird = 11;

        static const int NUMBER_OF_SENSOR_POSITIVES = 10;

        /* SENSOR VALUES */
        int frontLeftVal = 0;
        int frontRightVal = 0;
        int farRightVal = 0;
        int rightVals[NUMBER_OF_SENSOR_POSITIVES] = {0};
        int farLeftVal = 0;
        int leftVals[NUMBER_OF_SENSOR_POSITIVES] = {0};
        int backMiddleVal = 0;
        int middleVals[NUMBER_OF_SENSOR_POSITIVES] = {0};
        int distanceFrontVal = 0;
        int distanceVals[NUMBER_OF_SENSOR_POSITIVES] = {0};
        float distanceAvr;

        int lastSensorTriggered = 0;  // 0 for no idea, 1 for left, 2 for right

        /* MOTORS */
        float motorSpeed = 70; //EDIT THIS 
        float lineFollowDampingFactor = 0.9;
        float motorSpeedLeft;
        float motorSpeedRight;
        float speedDifference = 0;

        /* THRESHOLDS */
        int lineSensorThreshold = 300;
        float distanceSensorThreshold = 460;

        enum class ActionType {
        LINE, TURN_LEFT
        };
        ActionType currentRoutine = ActionType::LINE;

        void OnOffSwitch() {
            //sets start program to true at the push of the button
            int buttonState = digitalRead(startButtonPin);
            static bool lockSwitch = false;

            if (buttonState ==0 && lockSwitch==false) {
                run = run == true ? false : true;
                lockSwitch = true;
                if(run == true){
                    Serial.println("Running Program");
                    return;

                } else {
                    Serial.println("Pausing Program");
                    runMotors(0,0);
                    digitalWrite(ledPinFirst, LOW);
                    digitalWrite(ledPinSecond, LOW);
                    digitalWrite(ledPinThird,LOW);
                    return;
                }
            }
            if (buttonState == 1 && lockSwitch ==true) {
                lockSwitch = false;
            }
        }

        void runMotors(int motorLeftVal,int motorRightVal){
            int motorDirectionLeft = motorLeftVal > 0 ? BACKWARD : FORWARD;
            int motorDirectionRight = motorRightVal > 0 ? BACKWARD : FORWARD;
            if(motorLeftVal == 0 && motorRightVal == 0){
                motorDirectionLeft = 0;
                motorDirectionRight = 0;
            }
            //Serial.println("running");
            myMotorLeft->setSpeed(abs(motorLeftVal));
            myMotorLeft->run(motorDirectionLeft);
            myMotorRight->setSpeed(abs(motorRightVal));
            myMotorRight->run(motorDirectionRight);

            //Serial.println("motors running");
        }

        float checkAllSensorValues(bool listVals) {
            //Check ALL the sensor values

            frontLeftVal = analogRead(frontLeft);
            frontRightVal = analogRead(frontRight);
            
            //NEW METHOD OF ENSURING RELIABILITY (TLDR:TAKES AT LEAST 5 OF LAST 10 READINGS TO CHANGE)

            int leftSum = 0 ;
            int rightSum = 0;
            int middleSum = 0;
            int distanceSum = 0;
            
            for(int i = 1; i<NUMBER_OF_SENSOR_POSITIVES -1 ; i++ ){
                rightVals[i] = rightVals[i+1];
                leftVals[i] = leftVals[i+1];
                middleVals[i] = middleVals[i+1];
                distanceVals[i] = distanceVals[i+1];

                leftSum += rightVals[i];
                rightSum +=leftVals[i];
                middleSum +=middleVals[i];
                distanceSum += distanceVals[i];
            }
            rightVals[NUMBER_OF_SENSOR_POSITIVES -1] = digitalRead(offAxisLeft) ;
            leftVals[NUMBER_OF_SENSOR_POSITIVES -1] = digitalRead(offAxisRight) ;
            middleVals[NUMBER_OF_SENSOR_POSITIVES -1] = digitalRead(backMiddle) ;
            distanceVals[NUMBER_OF_SENSOR_POSITIVES -1] = digitalRead(distanceSensor); 

            leftSum += rightVals[NUMBER_OF_SENSOR_POSITIVES -1];
            rightSum +=leftVals[NUMBER_OF_SENSOR_POSITIVES -1];
            middleSum +=middleVals[NUMBER_OF_SENSOR_POSITIVES -1];
            distanceSum += distanceVals[NUMBER_OF_SENSOR_POSITIVES -1];

            farLeftVal = leftSum > 8 ? 1: 0; 
            farRightVal = rightSum > 8 ? 1: 0; 
            backMiddleVal = middleSum > 8 ? 1: 0; 
            distanceFrontVal = distanceSum > 8 ? 1: 0; 

            if(listVals){
                Serial.print("front left val:  ");
                Serial.println(frontLeftVal);
                Serial.print("front right val: ");
                Serial.println(frontRightVal);

                Serial.print("far right val:");
                Serial.println(farRightVal);
                Serial.print("far left val:");
                Serial.println(farLeftVal);
                Serial.print("back middle Val: ");
                Serial.println(backMiddleVal);
                Serial.print("distanceSensor: ");
                Serial.println(distanceFrontVal);
                Serial.println("                                     ");
                Serial.println("                                     ");
            }


            if(frontRightVal > lineSensorThreshold && frontLeftVal < lineSensorThreshold){
                lastSensorTriggered = 2;
            }
            if(frontLeftVal > lineSensorThreshold && frontRightVal < lineSensorThreshold){
                lastSensorTriggered = 1;
            }
        }

        void binaryFollowLine(int increaseRate) { 
            //COULD WE USE PROPORTIONAL CONTROL FOR THIS IE SPEED DIFFERENCE IS PROPORTIONAL TO LINESENSOR READING (POTENTIALLY ONLY IF ITS ABOVE THRESHOLD)
            //THIS WILL ALLOW US TO BE REALLY STRAIGHT ON THE STRAGHT BITS AND SO BLOCK PLACEMENT WILL BECOME SIMPLER...
            if(frontLeftVal > lineSensorThreshold && frontRightVal > lineSensorThreshold){
                while (frontLeftVal > lineSensorThreshold && frontRightVal > lineSensorThreshold){
                    /*
                    if(lastSensorTriggered == 2){ 
                        runMotors(-1*motorSpeed,1*motorSpeed);
                    } else if (lastSensorTriggered == 1) {
                        runMotors(1*motorSpeed,-1*motorSpeed);
                    }
                    */
                    runMotors(50,50);
                    checkAllSensorValues(false);
                }
            }
            else if (frontLeftVal > lineSensorThreshold) {
                speedDifference = increaseRate;
            }
            else if (frontRightVal > lineSensorThreshold) {
                speedDifference = -1 * increaseRate;
            }

            speedDifference *= lineFollowDampingFactor;
            motorSpeedLeft = motorSpeed - speedDifference;
            motorSpeedRight = motorSpeed + speedDifference;
            runMotors(motorSpeedLeft,motorSpeedRight);
            return;
        }

        void turnLeft() {
            //WAIT FOR FAR LEFT TO TRIGGER
            
            while(farLeftVal == 1 ){
                checkAllSensorValues(false);
                runMotors(-1*motorSpeed,1*motorSpeed);
            }

            while (farLeftVal == 0 ) {
                runMotors(-1*motorSpeed,1*motorSpeed); 
                checkAllSensorValues(false);
            }
            return;
        }

        void flashLED(){
            static int timer = 0;
            int state = LOW;
            timer += 1;
            if(timer > 100) timer = 1;
            Serial.println(timer);
            if(timer % 100 == 0 ){
              Serial.println("flashing");
                state = (state == HIGH) ? LOW : HIGH;
            }

            digitalWrite(ledPinFirst,state);
        }

        void chooseAction(){
            currentRoutine = ActionType::LINE;
            if(farLeftVal ==1 & farRightVal ==1 ){
                currentRoutine = ActionType::TURN_LEFT;
            }

            runAction();
        }

        void runAction(){
            switch(currentRoutine){
                case ActionType::LINE:
                    binaryFollowLine(70);
                    break;
                case ActionType::TURN_LEFT:
                    turnLeft();
                    break;
            }
        }
};

Robot Bot;

void setup() {

    AFMS.begin();
    Serial.begin(9600);

    pinMode(Bot.startButtonPin, INPUT);
    pinMode(Bot.offAxisLeft,INPUT);
    pinMode(Bot.offAxisRight,INPUT);

    pinMode(Bot.ledPinFirst,OUTPUT);
    pinMode(Bot.ledPinSecond,OUTPUT);
    pinMode(Bot.ledPinThird,OUTPUT);
}

void loop() {
    Bot.OnOffSwitch();
    if(run == true){
        Bot.flashLED();
        Bot.checkAllSensorValues(false);
        Bot.chooseAction();
    } else {
        Bot.runMotors(0,0);
    }
}
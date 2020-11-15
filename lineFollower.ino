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


class Robot {
    public:

        float orientation; //The orientation NESW of the robot - 0,N 90,E 180,S 360,W  

    /* Initialise sensor values*/

    int frontLeftVal = 0; //sensor values
    int frontRightVal = 0;
    int farRightVal = 0;
    int farLeftVal = 0;
    int backMiddleVal = 0;

    int distanceFrontVal = 0;

    /* end value initialisation */

    /* MOTORS */
    float motorSpeed = 70; //EDIT THIS 
    float motorSpeedLeft;
    float motorSpeedRight;
    float speedDifference = 0;

    int motorDirectionLeft = BACKWARD;
    int motorDirectionRight = BACKWARD;

    /* END MOTORS*/

    int lineSensorThreshold = 100;

    /*ALL PINS */
    int frontLeft = A0; // input pin for FRONT LEFT light sensor
    int frontRight = A1;
    int offAxisRight = A2;
    int offAxisLeft = A3;
    int backMiddle = A4;

    int startButtonPin = 2;

    int distanceSensor = A5;

    /*END PINS*/

    int FAR_RIGHT_THRESHOLD = 100;

    bool startProgram = true;

    enum class ActionType {
        LINE, TURN_LEFT, TURN_RIGHT, TURN_ONE_EIGHTY, PICKUP, BLUE_PLACE, RED_PLACE
    }; //ALL THE STATES OF THE ROBOT, ADD MORE IF NEEDED
    ActionType Action;

    void binaryFollowLine( int increaseRate) {
        static bool leftTurn = true;
        if (frontLeftVal > lineSensorThreshold) {
            speedDifference = increaseRate;
            leftTurn = true;
        }
        if (frontRightVal > lineSensorThreshold) {
            speedDifference = -1 * increaseRate;
            leftTurn = false;
        }
        /*
        else{
          if(leftTurn = true && speedDifference > increaseRate){
            speedDifference -= increaseRate;
          }
          else if(speedDifference < (-1 * increaseRate)) {
            speedDifference += increaseRate;
          }
        }
        if(abs(speedDifference)>motorSpeed){
            speedDifference = motorSpeed;
        }
        */

        //THE CONTROLLER SECTION
        motorSpeedLeft = motorSpeed - speedDifference;
        motorSpeedRight = motorSpeed + speedDifference;

        myMotorLeft->setSpeed(motorSpeedLeft);
        myMotorLeft->run(motorDirectionLeft);
        myMotorRight->setSpeed(motorSpeedRight);
        myMotorRight->run(motorDirectionRight);
    }

    float checkAllSensorValues() {

        //Check ALL the sensor values
        frontLeftVal = analogRead(frontLeft);
        frontRightVal = analogRead(frontRight);
        farRightVal = analogRead(offAxisRight);
        backMiddleVal = analogRead(backMiddle);

        distanceFrontVal = analogRead(distanceSensor);

        Serial.print("front left val:  ");
        Serial.println(frontLeftVal);
        Serial.print("front right val: ");
        Serial.println(frontRightVal);

        Serial.print("far right val:");
        Serial.println(farRightVal);
        Serial.print("back middle Val: ");
        Serial.println(backMiddleVal);
        Serial.print("distanceSensor: ");
        Serial.println(distanceFrontVal);
        Serial.println("                                     ");
        Serial.println("                                     ");
    }

    void decideActionToPerform() {
        //this takes the sensor values and works out which stage of the algorithm the robot needs to be in at any particular point

        //line follow until first junction
        //turn left 

    }

    void turnLeft() {
        //code here
    }

    void turnRight() {
        //code here
    }

    void turnOneEighty() {
        //code here
    }

    void turnInCircle() {
        //THE CONTROLLER SECTION
        motorSpeedLeft = motorSpeed;
        motorSpeedRight = motorSpeed;

        motorDirectionLeft = FORWARD;
        motorDirectionRight = BACKWARD;

        myMotorLeft->setSpeed(motorSpeedLeft);
        myMotorLeft->run(motorDirectionLeft);
        myMotorRight->setSpeed(motorSpeedRight);
        myMotorRight->run(motorDirectionRight);
    }

    void moveForward() {

        //THE CONTROLLER SECTION
        motorSpeedLeft = motorSpeed;
        motorSpeedRight = motorSpeed;

        myMotorLeft->setSpeed(motorSpeedLeft);
        myMotorLeft->run(motorDirectionLeft);
        myMotorRight->setSpeed(motorSpeedRight);
        myMotorRight->run(motorDirectionRight);
    }

    void runCurrentNeededAction() {
        switch (Action) {
            case ActionType::LINE:
                PIDfollowLine(2, 0.5, 0.5);
                break;
            case ActionType::TURN_LEFT:
                turnLeft();
                break;
            case ActionType::TURN_RIGHT:
                turnRight();
                break;
            case ActionType::PICKUP:
                turnRight();
                break;
            case ActionType::BLUE_PLACE:
                /* code */
                break;
            case ActionType::RED_PLACE:
                /* code */
                break;
            case ActionType::TURN_ONE_EIGHTY:
                turnOneEighty();
                break;

            default:
                break;
        }

    }

    void OnOffSwitch() {
        //sets start program to true at the push of the button
        int buttonState = digitalRead(2);

        if (buttonState == 1) {
            startProgram = startProgram == true ? false : true;
            while (buttonState == 1) {
                int buttonState = digitalRead(2);
            }
        }
    }

};

/*!
    @brief Initialise the pins for the arduino, and begin serial communications
*/

Robot Bot;


void setup() {

    AFMS.begin();
    Serial.begin(9600);

    pinMode(Bot.startButtonPin, INPUT);

}



/*!
    @brief This is the main control loop for THE ENTIRE PROJECT
*/
void loop() {
    Bot.OnOffSwitch();
    Bot.checkAllSensorValues();
    if(Bot.startProgram == true){
        //delay(1000);
        //Bot.PIDfollowLine(1,0.0,0.0);
        //Bot.binaryFollowLine(50);
        //Bot.turnInCircle();
        // Bot.moveForward();
    }

}
//this is a test program that should contrain the robot to follow a line.

//THE LOOP AT THE BOTTOM RUNS THE WHOLE PROGRAM
//SET THE CONTROLLER GAINS AT THE START on the line Robot Bot(2,0.5,0.5)
//these can be changed, eg for a pure proportional, choose (2,0,0)
//(proportional, integral, derivative)

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);

float motorSpeed = 200;

class Robot{
    public:
        Robot(float a, float b, float c){
            CONTROLLER_GAIN = a;
            INTEGRAL_GAIN = b;
            DERIVATIVE_GAIN = c;
        }

        float orientation; //The orientation NESW of the robot - 0,N 90,E 180,S 360,W  

        int frontLeftVal = 0;  // variable for light sensor 1
        int frontRightVal = 0;  // variable for light sensor 2
        int farRightVal = 0;  // variable for light sensor 3
        int backMiddleVal = 0;  // variable for light sensor 4

        float CONTROLLER_GAIN;
        float INTEGRAL_GAIN;
        float DERIVATIVE_GAIN;

        float motorSpeedLeft = 200;
        float motorSpeedRight = 200;
        float speedDifference = 0;

        int frontLeft = A0;    // input pin for FRONT LEFT light sensor
        int frontRight = A1;    // input pin for light sensor
        int offAxisRight = A2;    // input pin for light sensor
        int backMiddle = A3;    // input pin for light sensor

        enum class ActionType{LINE,TURN_LEFT,TURN_RIGHT,TURN_ONE_EIGHTY,PICKUP,BLUE_PLACE,RED_PLACE};  //ALL THE STATES OF THE ROBOT, ADD MORE IF NEEDED
        ActionType Action;


        void PIDfollowLine(){
            static float speedDiff;
            static float lastOffset;
            static float lineOffsetSum;

            int lineOffset = frontLeftVal-frontRightVal;
            lineOffsetSum = lineOffsetSum + lineOffset;

            int Integral = INTEGRAL_GAIN * lineOffsetSum;
            int Derivative = DERIVATIVE_GAIN * (lineOffset-lastOffset);
            int Proportional = CONTROLLER_GAIN * (lineOffset);

            lastOffset = lineOffset;

            //THE CONTROLLER SECTION
            speedDifference = Proportional + Integral + Derivative;
            motorSpeedLeft = speedDifference+motorSpeed;
            motorSpeedRight = speedDifference-motorSpeed;
            
            myMotor1->setSpeed(motorSpeedLeft);
            myMotor1->run(FORWARD);
            myMotor2->setSpeed(motorSpeedRight);
            myMotor2->run(FORWARD);
        }

        float checkAllSensorValues(){
            
            //Check ALL the sensor values
            frontLeftVal = analogRead(frontLeft);
            frontRightVal = analogRead(frontRight);
            farRightVal = analogRead(offAxisRight);
            backMiddleVal = analogRead(backMiddle);
        }

        void decideActionToPerform(){
            //this takes the sensor values and works out which stage of the algorithm the robot needs to be in at any particular point
        }

        void turnLeft(){

        }

        void turnRight(){

        }

        void turnOneEighty(){

        }

        void runCurrentNeededAction(){
            switch (Action)
            {
            case ActionType::LINE:
                PIDfollowLine();
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
};

/*!
    @brief Initialise the pins for the arduino, and begin serial communications
*/
void setup() {

    AFMS.begin();
    Serial.begin(9600); 
}

Robot Bot(2,0.5,0.5);

/*!
    @brief This is the main control loop for THE ENTIRE PROJECT
*/
void loop() {

    Bot.checkAllSensorValues();
    Bot.PIDfollowLine();

}

//this is a test program that should contrain the robot to follow a line.

//THE LOOP AT THE BOTTOM RUNS THE WHOLE PROGRAM
//SET THE CONTROLLER GAINS AT THE START on the line Robot Bot(2,0.5,0.5)
//these can be changed, eg for a pure proportional, choose (2,0,0)
//(proportional, integral, derivative)

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *myMotorRight = AFMS.getMotor(2);


class Robot{
    public:
    
        float orientation; //The orientation NESW of the robot - 0,N 90,E 180,S 360,W  

        int frontLeftVal = 0; //sensor values
        int frontRightVal = 0;  
        int farRightVal = 0;  
        int backMiddleVal = 0;  

        int distanceFrontVal = 0;

        float motorSpeed = 70; //EDIT THIS 
        float motorSpeedLeft;
        float motorSpeedRight;
        float speedDifference = 0;

        int motorDirectionLeft = BACKWARD;
        int motorDirectionRight = BACKWARD;

        int frontLeft = A0;    // input pin for FRONT LEFT light sensor
        int frontRight = A1;    // input pin for light sensor
        int offAxisRight = A2;    // input pin for light sensor
        int backMiddle = A3;    // input pin for light sensor

        int distanceSensor = A5;
        

        int FAR_RIGHT_THRESHOLD = 100;

        int startProgram = 1;



        enum class ActionType{LINE,TURN_LEFT,TURN_RIGHT,TURN_ONE_EIGHTY,PICKUP,BLUE_PLACE,RED_PLACE};  //ALL THE STATES OF THE ROBOT, ADD MORE IF NEEDED
        ActionType Action;


        void PIDfollowLine(int PROPORTIONAL_GAIN, int INTEGRAL_GAIN, int DERIVATIVE_GAIN){
            static float lastOffset;
            static int lineOffsetSum = 0;

            int lineOffset = frontLeftVal-frontRightVal;
            lineOffsetSum = lineOffsetSum + lineOffset;

            float Integral = INTEGRAL_GAIN * lineOffsetSum;
            float Derivative = DERIVATIVE_GAIN * (lineOffset-lastOffset);
            float Proportional = PROPORTIONAL_GAIN * (lineOffset);

            lastOffset = lineOffset;

            //THE CONTROLLER SECTION
            speedDifference = Proportional + Integral + Derivative;
            motorSpeedLeft = motorSpeed-speedDifference;
            motorSpeedRight = motorSpeed + speedDifference;
            
            myMotorLeft->setSpeed(motorSpeedLeft);
            myMotorLeft->run(motorDirectionLeft);
            myMotorRight->setSpeed(motorSpeedRight);
            myMotorRight->run(motorDirectionRight);
        }

        void binaryFollowLine(int threshold, int increaseRate){
            static bool leftTurn = true;
            if(frontLeftVal>threshold){
                speedDifference = increaseRate;
                leftTurn = true;
            }
            if(frontRightVal>threshold){
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
            motorSpeedLeft = motorSpeed-speedDifference;
            motorSpeedRight = motorSpeed + speedDifference;
            
            myMotorLeft->setSpeed(motorSpeedLeft);
            myMotorLeft->run(motorDirectionLeft);
            myMotorRight->setSpeed(motorSpeedRight);
            myMotorRight->run(motorDirectionRight);
        }

        float checkAllSensorValues(){
            
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
            
            Serial.print("far right val:" );
            Serial.println(farRightVal);            
            Serial.print("back middle Val: ");
            Serial.println(backMiddleVal);
            Serial.print("distanceSensor: ");
            Serial.println(distanceFrontVal);
            Serial.println("                                     ");
            Serial.println("                                     ");
        }

        void decideActionToPerform(){
            //this takes the sensor values and works out which stage of the algorithm the robot needs to be in at any particular point
        }

        void turnLeft(){
            //code here
        }

        void turnRight(){
            //code here
        }

        void turnOneEighty(){
            //code here
        }

        void turnInCircle(){
            //THE CONTROLLER SECTION
            motorSpeedLeft = motorSpeed;
            motorSpeedRight =  motorSpeed;

            motorDirectionLeft = FORWARD;
            motorDirectionRight = BACKWARD;
            
            myMotorLeft->setSpeed(motorSpeedLeft);
            myMotorLeft->run(motorDirectionLeft);
            myMotorRight->setSpeed(motorSpeedRight);
            myMotorRight->run(motorDirectionRight);
        }

        void moveForward(){
    
            //THE CONTROLLER SECTION
            motorSpeedLeft = motorSpeed;
            motorSpeedRight = motorSpeed;

            myMotorLeft->setSpeed(motorSpeedLeft);
            myMotorLeft->run(motorDirectionLeft);
            myMotorRight->setSpeed(motorSpeedRight);
            myMotorRight->run(motorDirectionRight);
        }

        void runCurrentNeededAction(){
            switch (Action)
            {
            case ActionType::LINE:
                PIDfollowLine(2,0.5,0.5);
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
        /*
        void OnOffSwitch(){
            int isOn = digitalRead(2);
            static int lock = 0;
            if(isOn == 1){
              startProgram = startProgram * -1;
              while (isOn == 1){
                delay(1);
             }
            }
    
           }
        */
};

/*!
    @brief Initialise the pins for the arduino, and begin serial communications
*/
void setup() {

    AFMS.begin();
    Serial.begin(9600); 

    pinMode(2, INPUT);

    pinMode(A5, INPUT);
}

Robot Bot;

/*!
    @brief This is the main control loop for THE ENTIRE PROJECT
*/
void loop() {
    //Bot.OnOffSwitch();
    Bot.checkAllSensorValues();
    delay(1000);
    //Bot.PIDfollowLine(1,0.0,0.0);
    //Bot.binaryFollowLine(100,50);
    //Bot.turnInCircle();
    // Bot.moveForward();
}

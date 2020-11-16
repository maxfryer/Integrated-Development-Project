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

        /* Initialise sensor values*/

        int frontLeftVal = 0; //sensor values
        int frontRightVal = 0;
        int farRightVal = 0;
        int farLeftVal = 0;
        int backMiddleVal = 0;

        int distanceFrontVal = 0;

        /* end value initialisation */

        /* MOTORS */
        float motorSpeed = 120; //EDIT THIS 
        float motorSpeedLeft;
        float motorSpeedRight;
        float speedDifference = 0;

        int motorDirectionLeft = BACKWARD;
        int motorDirectionRight = BACKWARD;

        /* END MOTORS*/

        int lineSensorThreshold = 100;
        int distanceSensorThreshold = 100;

        /*ALL PINS */
        int frontLeft = A0; // input pin for FRONT LEFT light sensor
        int frontRight = A1;
        int offAxisRight = A2;
        int offAxisLeft = A3;
        int backMiddle = A4;

        int startButtonPin = 2;

        int distanceSensor = A5;

        /*END PINS*/

        bool startProgram = true;

        enum class ActionType {
            LINE, TURN_LEFT, TURN_RIGHT, TURN_ONE_EIGHTY, PICKUP, BLUE_PLACE, RED_PLACE
        }; //ALL THE STATES OF THE ROBOT, ADD MORE IF NEEDED
        ActionType Action;

        enum class PositionList {
            START,FIRST_JUNCTION,MAIN_T_JUNCTION,BLUE_T_JUNCTION,PILL
        };
        PositionList position = PositionList::START;

        enum class TaskList {
            FIND_BOX,
            PLACE_FIRST_RED_BOX,
            PLACE_FIRST_BLUE_BOX,
            PLACE_SECOND_RED_BOX,
            PLACE_SECOND_BLUE_BOX,
            RETURN_HOME
        };
        TaskList currentTask = TaskList::FIND_BOX;

        int redBoxesCollected = 0;
        int blueBoxesCollected = 0;
        int boxColour = 0;

        void checkForNextLocation(){
            switch (position) {
                case PositionList::START:
                    if(farLeftVal > lineSensorThreshold){
                        position = PositionList::FIRST_JUNCTION;
                    }

                    break;
                case PositionList::FIRST_JUNCTION:
                    if(farLeftVal > lineSensorThreshold && farRightVal > lineSensorThreshold){
                        position = PositionList::MAIN_T_JUNCTION;
                    }
                    //check for the sensor positions that would give rise to the next state from here
                    break;
                case PositionList::MAIN_T_JUNCTION:
                    //check for the sensor positions that would give rise to the next state from here
                    if(farLeftVal < lineSensorThreshold && farRightVal < lineSensorThreshold){
                        position = PositionList:: PILL;
                    }
                    break;
                case PositionList::PILL:
                    break;
                case PositionList::BLUE_T_JUNCTION:
                    //check for the sensor positions that would give rise to the next state from here
                    break;
                default:
                    break;
            }
        }
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
            switch (currentTask) {
                case TaskList::FIND_BOX:
                    /*If at start, line follow to T-junction and then turn clockwise, travelling until a box is reached.
                    If at blue location, then line follow until mini T junction reached. Then travel to T junction and turn clockwise until box reached.
                    If at red location, then line follow until big T junction reached. Then turn clockwise and look for another box.
                    
                    
                    Then pick up the box and determine its colour. If blue then run blue placer
                    If red then run red placer. 
                    
                    */
                    break;
                case TaskList::PLACE_FIRST_BLUE_BOX:
                    /*
                    If box is blue then 180 degree turn and line follow until t junction hit. 
                    turn along path then turn right at first junction and line follow till blue T junction
                    turn clockwise and deposit box at first box location.
                    180 degree turn. Then run find box.
                    
                    */
                    break;
                case TaskList::PLACE_FIRST_RED_BOX:

                    break;
                case TaskList::PLACE_SECOND_BLUE_BOX:
                    /*If box is red then continue clockwise, avoiding boxes placed in the way, until a target location is reached. 
                    Then deposit box, 180 degree turn and continue go back to find box.
                    */
                    break;
                case TaskList::PLACE_SECOND_RED_BOX:
                    /*for second red box continue 
                    */ 
                    break;
                case TaskList::RETURN_HOME:
                    /* Wherever you are, line follow whilst avoiding obstacles until one of the t-junctions is reached. 
                    Then pick the right direction for going home, then line follow home.
                    Continue straight after the last junction until home you have reached the home line, then travel forward until your distance sensors hit the back wall, and reverse a bit.
                    Then stop!!
                    */
                    break;
            }
        }


        void pickUpBox(){
            //pick up box here
            if(boxColour == 'BLUE'){

            }
            else if (boxColour == 'RED'){

            }
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
                    binaryFollowLine(50);
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

Robot Bot;

void setup() {

    AFMS.begin();
    Serial.begin(9600);

    pinMode(Bot.startButtonPin, INPUT);
    

}


void loop() {
    Bot.OnOffSwitch();
    Bot.checkAllSensorValues();
    if(Bot.startProgram == true){
        //delay(1000);
        //Bot.PIDfollowLine(1,0.0,0.0);
        //Bot.binaryFollowLine(50);
        //Bot.turnInCircle();
        // Bot.moveForward();
        Bot.checkForNextLocation();
        Bot.decideActionToPerform();
        Bot.runCurrentNeededAction();
    }

}
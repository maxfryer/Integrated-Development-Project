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


        /*ALL PINS */
        int frontLeft = A0; // input pin for FRONT LEFT light sensor
        int frontRight = A1;
        int offAxisRight = 3;
        int offAxisLeft = 4;
        int backMiddle = 5;

        int startButtonPin = 2;

        int distanceSensor = A5;

        /*END PINS*/
        /* Initialise sensor values*/

        int frontLeftVal = 0; //sensor values
        int frontRightVal = 0;
        int farRightVal = 0;
        int farLeftVal = 0;
        int backMiddleVal = 0;

        int distanceFrontVal = 0;

        /* end value initialisation */

        /* MOTORS */
        float motorSpeed = 150; //EDIT THIS 
        float motorSpeedLeft;
        float motorSpeedRight;
        float speedDifference = 0;


        /* END MOTORS*/

        int lineSensorThreshold = 300;
        int distanceSensorThreshold = 100;

        

        bool startProgram = false;

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

        //int timer = 0; 16/11 I THINK WE COULD ADD A REDUNDENCY TIMER TO PROCESSES SO THEY DONT CONTINUE
        //FOREVER IN CASE OF SENSOR FAILURE

        void checkForNextLocation(){
            
            static PositionList lastPosition = PositionList::START;
            static char place[20] = "start";
            

            if(position != lastPosition){
                Serial.println(place);
            }
            lastPosition = position;
            
            switch (position) {
                case PositionList::START:
                    
                    if(farLeftVal ==1 ){
                        position = PositionList::FIRST_JUNCTION;
                        strcpy(place,"junction1");
                    }

                    break;
                case PositionList::FIRST_JUNCTION:
                    
                    if(farLeftVal ==1  && farRightVal ==1 ){
                        position = PositionList::MAIN_T_JUNCTION;
                        strcpy(place,"mainJunc");
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
                    strcpy(place,"pill");
                    break;
                case PositionList::BLUE_T_JUNCTION:
                    strcpy(place,"blueTJunc");
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

            runMotors(motorSpeedLeft,motorSpeedRight);
        }

        float checkAllSensorValues(bool listVals) {

            //Check ALL the sensor values
            frontLeftVal = analogRead(frontLeft);
            frontRightVal = analogRead(frontRight);
            farRightVal = digitalRead(offAxisRight);
            farLeftVal = digitalRead(offAxisLeft);
            backMiddleVal = digitalRead(backMiddle);

            distanceFrontVal = analogRead(distanceSensor);

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
 
        }

        void decideActionToPerform() {
            switch (currentTask) {
                case TaskList::FIND_BOX:
                    Action = ActionType::LINE;
                    if(position == PositionList::MAIN_T_JUNCTION){
                        Action = ActionType::TURN_LEFT;
                    }
                    if(position == PositionList::PILL){
                        static int numTargetLocationPassed = 0;
                        if((farLeftVal == 1  || farRightVal == 0 ) && Action == ActionType::LINE ){
                            /*if(boxTargetLocation < 2){}
                                numTargetLocationPassed +=1;
                                Serial.println("passed target location")
                            } else {*/
                                position == PositionList::MAIN_T_JUNCTION;
                                Action = ActionType::TURN_LEFT;
                            //}
                            
                            //congrats you've found a box, increment. If increment already equals 2 then you've hit the T junction again
                        }
                    }
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

        void turnLeft() {            
            //THE CONTROLLER SECTION


            runMotors(-1*motorSpeed,1*motorSpeed);

            while (Action == ActionType::TURN_LEFT){
                //WAIT FOR FAR LEFT TO TRIGGER
                if (farLeftVal == 1) {
                motorSpeedLeft = 0;
                motorSpeedRight = 0;

                break;
                }
            }
            Action == ActionType::LINE;


        }

        void turnRight() {            
            //THE CONTROLLER SECTION


            runMotors(1*motorSpeed,-1*motorSpeed);

            while (Action == ActionType::TURN_RIGHT){
                //WAIT FOR FAR RIGHT TO TRIGGER
                if (farRightVal == 1) {
                motorSpeedLeft = 0;
                motorSpeedRight = 0;

                break;
                }
            }

           Action == ActionType::LINE;
           position = PositionList::PILL;

        }

        void turnOneEighty() {
            //THE CONTROLLER SECTION
            runMotors(-1*motorSpeed,motorSpeed);

            while (Action == ActionType::TURN_ONE_EIGHTY){
                //WAIT FOR BACK TO TRIGGER BUT ONLY IF RIGHT IS NOT ALSO TRIGGERED
                if (backMiddleVal == 1 && farRightVal == 1) {
                motorSpeedLeft = 0;
                motorSpeedRight = 0;

                break;
                }
            }

            runMotors(-1*motorSpeedLeft,-1*motorSpeedRight);

        }

        void turnInCircle() {
            //THE CONTROLLER SECTION
            runMotors(-1*motorSpeed,motorSpeed);
        }

        void moveForward() {
            runMotors(motorSpeed,motorSpeed);
        }

        void runCurrentNeededAction() {

            //testing 
            //Action = ActionType::TURN_LEFT;
            //testing 
            switch (Action) {
                case ActionType::LINE:
                    binaryFollowLine(100);
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
<<<<<<< HEAD
            static bool lockSwitch = false;
            if (digitalRead(startButtonPin)==1 && lockSwitch==false) {
                startProgram = startProgram == true ? false: true;
=======
            static bool lockSwitch = false
            if (digitalRead(startButtonPin)==1 && lockSwitch=false) {
                startProgram = !startProgram;
>>>>>>> 79be79809a66063f55913cf54dcf5d3cd1d11ab2
                lockSwitch = true;
            } else if (digitalRead(startButtonPin)==0 && lockSwitch ==true) {
                lockSwitch = false;
            }
            
            //Serial.println(buttonState);
            /*
            if (buttonState == 1) {
                startProgram = (startProgram == true) ? false : true;
                if (startProgram == true) {Serial.println("starting");
                }
                else if(startProgram == false) Serial.println("finishing");
                while (buttonState == 1) {
                    int buttonState = digitalRead(startButtonPin);
                }
            }*/
            
        }

        void runMotors(int motorLeftVal,int motorRightVal){
            int motorDirectionLeft = motorLeftVal > 0 ? BACKWARD : FORWARD;
            int motorDirectionRight = motorRightVal > 0 ? BACKWARD : FORWARD;
            //Serial.println("running");
            myMotorLeft->setSpeed(abs(motorLeftVal));
            myMotorLeft->run(motorDirectionLeft);
            myMotorRight->setSpeed(abs(motorRightVal));
            myMotorRight->run(motorDirectionRight);
        }

};



Robot Bot;

void setup() {

    AFMS.begin();
    Serial.begin(9600);

    pinMode(Bot.startButtonPin, INPUT);
    pinMode(Bot.offAxisLeft,INPUT);
    pinMode(Bot.offAxisRight,INPUT);
}


void loop() {
    Bot.OnOffSwitch();
    Bot.checkAllSensorValues(false);
    //delay(1000);
    if(Bot.startProgram == true){
        //delay(1000);
        //Bot.binaryFollowLine(100);
        //Bot.turnInCircle();
        // Bot.moveForward();
        Bot.checkForNextLocation();
        Bot.decideActionToPerform();
        Bot.runCurrentNeededAction();
    }

}

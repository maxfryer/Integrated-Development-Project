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

        /* SENSOR VALUES */
        int frontLeftVal = 0; //sensor values
        int frontRightVal = 0;
        int farRightVal = 0;
        int farLeftVal = 0;
        int backMiddleVal = 0;
        int distanceFrontVal = 0;
        int distanceThreshold = 0;

        /* MOTORS */
        float motorSpeed = 70; //EDIT THIS 
        float motorSpeedLeft;
        float motorSpeedRight;
        float speedDifference = 0;

        /* THRESHOLDS */
        int lineSensorThreshold = 300;
        int distanceSensorThreshold = 100;

        /* SUBROUTINES */
        enum class ActionType {
            LINE, TURN_LEFT, TURN_RIGHT, TURN_ONE_EIGHTY, DETECT, PICKUP, BLUE_PLACE, RED_PLACE,
            CONTROL_ONE, CONTROL_TWO, CONTROL_TEST
        }; //ALL THE STATES OF THE ROBOT, ADD MORE IF NEEDED
        ActionType Action = ActionType::CONTROL_ONE

        /* POSITIONS */
        enum class PositionList {
            START,FIRST_JUNCTION,TUNNEL,MAIN_T_JUNCTION,PILL,
            BLUE_TRACK,BLUE_BOX_BOTTOM,BLUE_BOX_BOTTOM_RIGHT,BLUE_BOX_RIGHT,
            BLUE_BOX_TOP_RIGHT, BLUE_BOX_TOP, BLUE_T_JUNCTION
        };
        PositionList position = PositionList::START;

        /* DIRECTIONS */
        enum class Directions {TOWARDS_PILL,AWAY_FROM_PILL};
        Directions direction = Directions::TOWARDS_PILL;

        /* VARIABLES */
        int processTime = 0;
        bool run = false;
        int redBoxesCollected = 0;
        int blueBoxesCollected = 0;
        enum class boxColours {NONE,RED,BLUE};
        boxColours boxColour = boxColours::NONE;

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

        void checkForNextLocation(){
            
            static PositionList lastPosition = PositionList::START;
            static char place[20] = "start";
            

            if(position != lastPosition){
                Serial.println(place);
            }
            lastPosition = position;
            
            
            if(Action == ActionType::LINE){  
                switch (position) {
                    case PositionList::START:
                        if((farLeftVal == 1) && (direction == Directions::TOWARDS_PILL)){
                            position = PositionList::TUNNEL;
                            strcpy(place,"reached junction1");
                        }
                        break;

                    case PositionList::FIRST_JUNCTION:

                        if((farLeftVal == 0) && (direction == Directions::TOWARDS_PILL)){
                            position = PositionList::TUNNEL;
                            strcpy(place,"on tunnel track towards pill");
                        }

                        break;

                    case PositionList::TUNNEL:
                        
                        if((farLeftVal == 1  && farRightVal == 1) && (direction == Directions::TOWARDS_PILL) ){
                            position = PositionList::MAIN_T_JUNCTION;
                            strcpy(place,"reached mainJunc");
                        }
                        if((direction == Directions::AWAY_FROM_PILL) && (farRightVal == 1)){
                            position = PositionList::FIRST_JUNCTION;
                        }

                        //check for the sensor positions that would give rise to the next state from here
                        break;
                
                }
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



        void decideActionToPerform() {
            switch (currentTask) {
                case TaskList::FIND_BOX:
                    static bool clockwise = true;
                    Action = ActionType::LINE;
                    if(position == PositionList::MAIN_T_JUNCTION){
                        if(clockwise = true){
                            Action = ActionType::TURN_LEFT;
                        }
                        if(clockwise = false){
                            Action = ActionType::TURN_RIGHT;
                        }
                    }
                    if(position == PositionList::PILL){
                        static bool onTargetBox; 
                        static int numTargetLocationPassed = 0;
                        if((farLeftVal == 1  || farRightVal == 1 ) && Action == ActionType::LINE ){
                            if(numTargetLocationPassed < 2){
                                if(onTargetBox == false){
                                    numTargetLocationPassed +=1;
                                    Serial.println("passed target location");
                                    onTargetBox = true;
                                }
                            } else {
                                position == PositionList::MAIN_T_JUNCTION; //should this be single =
                                Action = ActionType::TURN_LEFT; //is this for if robot goes all the way around pill?
                            }
                        } else {
                            onTargetBox = false;
                        }

                        if(distanceFrontVal < distanceThreshold){
                            //this is detecting whether to pick up the box or not
                            currentTask = TaskList::DETECT_COLOUR;
                            if(boxColour == boxColours::RED && clockwise == true && blueBoxesCollected < 2){
                                currentTask = TaskList::REJECT_RED_BOX
                                
                            } else {
                                if(blueBoxesCollected == 0){
                                    currentTask = TaskList::PLACE_FIRST_BLUE_BOX;
                                } else {
                                    currentTask = TaskList::PLACE_SECOND_BLUE_BOX;
                                }
                            }
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
                    if (position == PositionList::BLUE_T_JUNCTION && direction == Directions::AWAY_FROM_PILL){
                        Action = ActionType::TURN_LEFT;
                    }
                    if ((farLeftVal == 1  || farRightVal == 1 ) && position == PositionList::BLUE_BOX_BOTTOM && direction == Directions::AWAY_FROM_PILL){
                        Action = ActionType::BLUE_PLACE;
                    }
                    if (position == PositionList::BLUE_T_JUNCTION && direction == Directions::TOWARDS_PILL){
                        Action = ActionType::TURN_RIGHT;
                    }
                    // at the end go back to finding a box
                    if (position == PositionList::BLUE_TRACK && direction == Directions::TOWARDS_PILL){
                        currentTask = TaskList::FIND_BOX;
                    }

                    break;
                    
                case TaskList::PLACE_FIRST_RED_BOX:
                    // at the end go back to finding a box
                    currentTask = TaskList::FIND_BOX;
                    break;

                case TaskList::PLACE_SECOND_BLUE_BOX:
                    /*If box is red then continue clockwise, avoiding boxes placed in the way, until a target location is reached. 
                    Then deposit box, 180 degree turn and continue go back to find box.
                    */
                    if (position == PositionList::BLUE_T_JUNCTION && direction == Directions::AWAY_FROM_PILL){
                        Action = ActionType::TURN_RIGHT;
                    }
                    if (farLeftVal == 1 && position == PositionList::BLUE_BOX_BOTTOM_RIGHT && direction == Directions::AWAY_FROM_PILL){
                        Action = ActionType::TURN_LEFT;
                    }
                    if (farLeftVal == 1 && position == PositionList::BLUE_BOX_TOP_RIGHT && direction == Directions::AWAY_FROM_PILL){
                        Action = ActionType::TURN_LEFT;
                    }
                    if ((farLeftVal == 1 || farRightVal == 1 ) && position == PositionList::BLUE_BOX_TOP && direction == Directions::AWAY_FROM_PILL){
                        Action = ActionType::BLUE_PLACE;
                    }
                    if (farRightVal == 1 && position == PositionList::BLUE_BOX_TOP_RIGHT && direction == Directions::TOWARDS_PILL){
                        Action = ActionType::TURN_RIGHT;
                    }
                    if (farRightVal == 1 && position == PositionList::BLUE_BOX_BOTTOM_RIGHT && direction == Directions::TOWARDS_PILL){
                        Action = ActionType::TURN_RIGHT;
                    }
                    if (farLeftVal == 1 && position == PositionList::BLUE_BOX_BOTTOM && direction == Directions::TOWARDS_PILL){
                        Action = ActionType::TURN_LEFT;
                    }
                    // at the end go back to finding a box
                    if (position == PositionList::BLUE_TRACK && direction == Directions::TOWARDS_PILL){
                        currentTask = TaskList::FIND_BOX;
                    }
                    break;

                case TaskList::PLACE_SECOND_RED_BOX:
                    /*for second red box continue 
                    */ 
                    
                    // at the end go back to finding a box
                    currentTask = TaskList::FIND_BOX;
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
            runMotors(-1*motorSpeed,1*motorSpeed);
            //WAIT FOR FAR LEFT TO TRIGGER
            if (farLeftVal == 1) {
                Action == ActionType::LINE;
                if(position == PositionList::MAIN_T_JUNCTION && direction == Directions::TOWARDS_PILL){
                    position = PositionList::PILL;
                }
                if(position == PositionList::MAIN_T_JUNCTION && direction == Directions::AWAY_FROM_PILL){
                    position = PositionList::TUNNEL;
                }
                if(position == PositionList::BLUE_T_JUNCTION && direction == Directions::AWAY_FROM_PILL){
                    position = PositionList::BLUE_BOX_BOTTOM;
                }
                if(position == PositionList::BLUE_T_JUNCTION && direction == Directions::TOWARDS_PILL){
                    position = PositionList::BLUE_TRACK;
                }
                if(position == PositionList::BLUE_BOX_BOTTOM_RIGHT && direction == Directions::AWAY_FROM_PILL){
                    position = PositionList::BLUE_BOX_RIGHT;
                }
                if(position == PositionList::BLUE_BOX_TOP_RIGHT && direction == Directions::AWAY_FROM_PILL){
                    position = PositionList::BLUE_BOX_TOP;
                }
                Serial.println("done left junction");
                return;
            }
        }

        void turnRight() {           

            while (Action == ActionType::TURN_RIGHT){
                runMotors(1*motorSpeed,-1*motorSpeed);
                //WAIT FOR FAR RIGHT TO TRIGGER
                if (farRightVal == 1) {
                runMotors(motorSpeed,motorSpeed);
                Action == ActionType::LINE;
                Serial.println("done right junction");
                return;
                }
            }
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

            switch (Action) {
                case ActionType::LINE:
                    binaryFollowLine(70);
                    break;
                case ActionType::TURN_LEFT:
                    turnLeft();
                    break;
                case ActionType::TURN_RIGHT:
                    turnRight();
                    break;
                case ActionType::PICKUP:
                    pickupBox();
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

        void pickupBox(){
            //picks up the box AND AND AND checks it colour
        }

        void OnOffSwitch() {
            //sets start program to true at the push of the button
            int buttonState = digitalRead(startButtonPin);

            static bool lockSwitch = false;
            if (buttonState ==1 && lockSwitch==false) {
                run = run == true ? false : true;
                lockSwitch = true;
                if(run = true){
                    Serial.println("Running Program");
                    switch (blueBoxesCollected) {
                        case 0:
                            action = ActionType::
                    }
                } else {
                    Serial.println("Pausing Program");
                    runMotors(0,0);
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
    Bot.checkForNextLocation();
    Bot.decideActionToPerform();
    Bot.runCurrentNeededAction();
}


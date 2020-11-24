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
        

        int ledPinFirst = 9;
        int ledPinSecond = 10;
        int ledPinThird = 8;

        static const int NUMBER_OF_SENSOR_POSITIVES = 10;

        /* SENSOR VALUES */
        int frontLeftVal = 0; //sensor values
        
        int frontRightVal = 0;
        int farRightVal = 0;
        int rightVals[NUMBER_OF_SENSOR_POSITIVES];
        int farLeftVal = 0;
        int leftVals[NUMBER_OF_SENSOR_POSITIVES];
        int backMiddleVal = 0;
        int distanceFrontVal = 0;

        /* MOTORS */
        float motorSpeed = 70; //EDIT THIS 
        float lineFollowDampingFactor = 0.9;
        float motorSpeedLeft;
        float motorSpeedRight;
        float speedDifference = 0;

        /* THRESHOLDS */
        int lineSensorThreshold = 300;
        int distanceSensorThreshold = 100;

        /* SUBROUTINES */
        
        /* POSITIONS */
        enum class PositionList {
            START,TUNNEL,PILL,
            BLUE_TRACK,BLUE_BOX_BOTTOM,BLUE_BOX_RIGHT,
            BLUE_BOX_TOP,
        };
        PositionList position = PositionList::TUNNEL;

        /* DIRECTIONS */
        enum class Directions {TOWARDS_PILL,AWAY_FROM_PILL};
        Directions direction = Directions::TOWARDS_PILL;

        /* VARIABLES */
        
        // 0 -- Test left turns 
        // 1 -- Test right turns
        // 2 -- 180 degree turns
        // 3 -- line follow and detect
        // 4 -- line follow and detect and pickup if blue
        // 5 -- line follow and detect and reverse if red
        // 6 -- line follow round pill then turn back to start
        int testNumber = 0;
        bool testMode = true;
        int boxesCollected = 0;
        bool clockwise = true;
        bool onTargetBox; 
        int numTargetLocationPassed = 0;

        enum class boxColours {NONE,RED,BLUE};
        boxColours boxColour = boxColours::NONE;

        enum class ActionType {
        CONTROL_TEST_ZERO, CONTROL_TEST_ONE, CONTROL_TEST_TWO,
        CONTROL_TEST_THREE, CONTROL_TEST_FOUR, CONTROL_TEST_FIVE, CONTROL_TEST_SIX, 
        DECIDE_CONTROL, TURN_HOME, TURN_LEFT, TURN_RIGHT, TURN_ONEEIGHTY,
        CONTROL_ONE, CONTROL_TWO, 
        }; //ALL THE STATES OF THE ROBOT, ADD MORE IF NEEDED
        ActionType currentRoutine = ActionType::DECIDE_CONTROL;

        void OnOffSwitch() {
            //sets start program to true at the push of the button
            int buttonState = digitalRead(startButtonPin);
            static bool lockSwitch = false;

            if (buttonState ==0 && lockSwitch==false) {
                run = run == true ? false : true;
                lockSwitch = true;
                if(run == true){
                    Serial.println("Running Program");
                    digitalWrite(ledPinFirst, HIGH);
                    digitalWrite(ledPinSecond, HIGH);
                    digitalWrite(ledPinThird,HIGH);

                    currentRoutine = ActionType::DECIDE_CONTROL;
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

        float checkAllSensorValues(bool listVals) {
            //Check ALL the sensor values
            farLeftVal = 1;
            for (int i = 0; i <NUMBER_OF_SENSOR_POSITIVES-1; i++){
                leftVals[i] = leftVals[i+1];
                if(leftVals[i] == 0){
                    farLeftVal = 0;
                }
            }
            leftVals[NUMBER_OF_SENSOR_POSITIVES-1] = digitalRead(offAxisLeft);
            if(farLeftVal == 1){
                farLeftVal = leftVals[NUMBER_OF_SENSOR_POSITIVES-1];
            }


            farRightVal = 1;
            for (int i = 0; i <NUMBER_OF_SENSOR_POSITIVES-1; i++){
                rightVals[i] = rightVals[i+1];
                if(rightVals[i] == 0){
                    farRightVal = 0;
                }
            }
            rightVals[NUMBER_OF_SENSOR_POSITIVES-1] = digitalRead(offAxisRight);
            if(farRightVal == 1){
                farRightVal = rightVals[NUMBER_OF_SENSOR_POSITIVES-1];
            }


            frontLeftVal = analogRead(frontLeft);
            frontRightVal = analogRead(frontRight);
            
            backMiddleVal = digitalRead(backMiddle);
            distanceFrontVal = analogRead(distanceSensor);

            //NEW METHOD OF ENSURING RELIABILITY (TLDR:TAKES AT LEAST 5 OF LAST 10 READINGS TO CHANGE)
            static int farLeftTotal = 0;
            static int farRightTotal = 0;

            for(int i = 0; i <NUMBER_OF_SENSOR_POSITIVES-2; i++){
                //shifts last 9 readings
                leftVals[i] = leftVals[i+1];
                rightVals[i] = rightVals[i+1];
            }

            //adds most recent reading
            leftVals[NUMBER_OF_SENSOR_POSITIVES-1] = digitalRead(offAxisLeft);
            rightVals[NUMBER_OF_SENSOR_POSITIVES-1] = digitalRead(offAxisRight);

            for(int i = 0; i <NUMBER_OF_SENSOR_POSITIVES-1; i++){
                //adds total number of 1s (and by extension 0s)
                farLeftTotal += leftVals[i];
                farRightTotal += rightVals[i];
            }

            //N.B "5" MUST BE CHANGED TO HALF THE NUMBER OF SENSORS
            if(farLeftTotal>5){
                farLeftVal = 1;
            }
            if(farLeftTotal<5){
                farLeftVal= 0;
            }
            if(farRightTotal>5){
                farRightVal = 1;
            }
            if(farRightTotal<5){
                farRightVal = 0;
            }
            //IF 5 DOENSN'T CHANGE...

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
            //ENSURES LOCATION IS KEPT UP TO DATE WHEN FOLLOWING A LINE MAYBE COULD BE INTEGRATED WITH LINE FOLLOWING CODE, IN FACT PROBABLY SHOULD BE
            static PositionList lastPosition = PositionList::START;
            
            if(position==PositionList::START && (farLeftVal == 1) && (direction == Directions::TOWARDS_PILL)){
                position = PositionList::TUNNEL;
                Serial.println("Reached Tunnel");
                return;
            }
            if(position == PositionList::BLUE_TRACK && farRightVal == 1 && direction == Directions::TOWARDS_PILL){
                position = PositionList::TUNNEL;
                Serial.println("Reached Tunnel");
                return;
            }
            if(position == PositionList::TUNNEL && direction == Directions::AWAY_FROM_PILL && farLeftVal == 1){
                position = PositionList::BLUE_TRACK;
                Serial.println("Reached Blue Track");
                return;
            }
        }

        void subRoutine() {
            switch(currentRoutine) {
                case ActionType::DECIDE_CONTROL:
                    //if test mode activated will go straight there without considering no. boxes collected
                    if(testMode == true){
                        if(testNumber == 0){
                            currentRoutine = ActionType::CONTROL_TEST_ZERO;
                            Serial.println("Continuing to Test Loop 0");
                            break;
                        }
                        if(testNumber == 1){
                            currentRoutine = ActionType::CONTROL_TEST_ONE;
                            Serial.println("Continuing to Test Loop 1");
                            break;
                        }
                        if(testNumber == 2){
                            currentRoutine = ActionType::CONTROL_TEST_TWO;
                            Serial.println("Continuing to Test Loop 2");
                            break;
                        }
                        if(testNumber == 6){
                            currentRoutine = ActionType::CONTROL_TEST_SIX;
                            Serial.println("Continuing to Test Loop 6");
                            break;
                        }
                        else{
                            Serial.println("Reached end of Decide Control and nothing has been Decided");
                        }
                    }else {
                        if(boxesCollected == 0){
                            currentRoutine = ActionType::CONTROL_ONE;
                            break;
                        }
                        if(boxesCollected == 1){
                            currentRoutine = ActionType::CONTROL_TWO;
                            break;
                        }
                    }
                case ActionType::CONTROL_TEST_ZERO:
                    //TESTS TURNING LEFT
                    if(position == PositionList::TUNNEL && direction == Directions::TOWARDS_PILL && farLeftVal == 1 && farRightVal == 1){
                        Serial.println("Control Loop 0 - TURN LEFT ONTO PILL");
                        currentRoutine = ActionType::TURN_LEFT;
                        break;
                    }else {
                        followLine();
                        //n.b followLine includes binaryfollowline(70)
                        break;
                    }
                case ActionType:: CONTROL_TEST_ONE:
                    //TESTS TURNING RIGHT
                    if(position == PositionList::TUNNEL && direction == Directions::TOWARDS_PILL && farLeftVal == 1 && farRightVal == 1){
                        Serial.println("Control Loop 1 - TURN RIGHT ONTO PILL");
                        currentRoutine = ActionType::TURN_RIGHT;
                        break;
                    }else {
                        followLine();
                        //n.b followLine includes binaryfollowline(70)
                        break;
                    }
                case ActionType::CONTROL_TEST_TWO:
                    //TESTS TURNING FULL 180 DEGREES
                    if(position == PositionList::TUNNEL && direction == Directions::TOWARDS_PILL && distanceFrontVal> distanceSensorThreshold){
                        Serial.println("Control Loop 2 - TURN 180");
                        currentRoutine = ActionType::TURN_ONEEIGHTY;
                        break;
                    }else {
                        followLine();
                        //n.b followLine includes binaryfollowline(70)
                        break;
                    }
                case ActionType::CONTROL_TEST_SIX:
                
                    if(position == PositionList::TUNNEL && direction == Directions::TOWARDS_PILL && farLeftVal == 1 && farRightVal == 1){
                        Serial.println("Turn onto pill");
                        currentRoutine = ActionType::TURN_LEFT;
                        break;
                    }
                    if(position == PositionList::PILL && numTargetLocationPassed == 2 && farLeftVal == 1){
                        Serial.println("looping");
                        currentRoutine = ActionType::TURN_LEFT;
                        break;
                    }
                    if(position == PositionList::TUNNEL && direction == Directions::AWAY_FROM_PILL && (frontRightVal > lineSensorThreshold && frontLeftVal > lineSensorThreshold)){
                        currentRoutine = ActionType::TURN_HOME;
                        break;
                    }
                    if(position == PositionList::START && direction == Directions::AWAY_FROM_PILL && frontRightVal < lineSensorThreshold && frontLeftVal < lineSensorThreshold){
                        Serial.println("Finished Program");
                        run = false;
                        break;
                    }else {
                        followLine();
                        //n.b followLine includes binaryfollowline(70)
                        break;
                    }
                case ActionType::TURN_LEFT:
                    turnLeft();
                    break;
                case ActionType::TURN_RIGHT:
                    turnRight();
                    break;
                case ActionType::TURN_ONEEIGHTY:
                    turnAround();
                    break;
                case ActionType::TURN_HOME:
                    turnHome(70);
                    break;
                    
            

            }
        }

        void followLine() {
            //what to do if robot goes over target box note sensors must be in line for this code to work
            //incorperate in decision code
            if(position == PositionList::PILL) {
                if(onTargetBox == true){
                    if((frontLeftVal > lineSensorThreshold || frontRightVal > lineSensorThreshold) && farRightVal == 0 && farLeftVal == 0) {
                        onTargetBox = false;
                        return;
                    }
                    else {
                        return;
                    }
                }
                else if(farLeftVal == 1 && farRightVal == 1){
                    //Might not trigger if very angled, see proportional control though was originally 'or' not 'and'
                    Serial.println("On Target Location");
                    onTargetBox = true;
                    numTargetLocationPassed +=1;
                    Serial.println("reached a target spot");
                    runMotors(motorSpeed,motorSpeed);
                    return;
                }
                else{
                    binaryFollowLine(70);
                    return;
                }
            }
            else {
                binaryFollowLine(70);
                return;
            }
        }

        void binaryFollowLine(int increaseRate,int ignoreSide = 0) { 
            //COULD WE USE PROPORTIONAL CONTROL FOR THIS IE SPEED DIFFERENCE IS PROPORTIONAL TO LINESENSOR READING (POTENTIALLY ONLY IF ITS ABOVE THRESHOLD)
            //THIS WILL ALLOW US TO BE REALLY STRAIGHT ON THE STRAGHT BITS AND SO BLOCK PLACEMENT WILL BECOME SIMPLER...
            if(ignoreSide != 2){
                if (frontLeftVal > lineSensorThreshold) {
                    speedDifference = increaseRate;
                }
            }
            if(ignoreSide !=1) {
                if (frontRightVal > lineSensorThreshold) {
                    speedDifference = -1 * increaseRate;
                }
            }
            speedDifference *= lineFollowDampingFactor;
            motorSpeedLeft = motorSpeed - speedDifference;
            motorSpeedRight = motorSpeed + speedDifference;
            runMotors(motorSpeedLeft,motorSpeedRight);
            return;
        }

        void turnAround() {
            return;
        }

        void turnRight() {
            return;
        }

        void turnLeft() {
            //WAIT FOR FAR LEFT TO TRIGGER

            static bool leftLine =false;
            if(farLeftVal == 1 && leftLine == false){
                runMotors(-1*motorSpeed,1*motorSpeed);
            }
            if(farLeftVal == 0 && leftLine == false){
                leftLine = true;
            }
            if (farLeftVal == 1 && leftLine == true) {
                currentRoutine = ActionType::DECIDE_CONTROL;
                //runMotors(0,0);
                leftLine = false;
                
                if(position == PositionList::TUNNEL && direction == Directions::TOWARDS_PILL){
                    position = PositionList::PILL;
                    Serial.println("Joined Pill");
                    return;
                }
                else if(position == PositionList::PILL){
                    position = PositionList::TUNNEL;
                    direction = Directions::AWAY_FROM_PILL;
                    Serial.println("Turned onto Tunnel");
                    return;
                }
            }          
            return;
        }

        void turnHome(int increaseRate) {
            //ONLY DIFFERENCE IS ORDER IN WHICH VALUES ARE CHECKED SO THAT LEFT TURNING IS FAVOURED
            if (frontRightVal > lineSensorThreshold) {
                speedDifference = -1 * increaseRate;
            }
            if (frontLeftVal > lineSensorThreshold) {
                speedDifference = increaseRate;
            }
            motorSpeedLeft = motorSpeed - speedDifference;
            motorSpeedRight = motorSpeed + speedDifference;
            runMotors(motorSpeedLeft,motorSpeedRight);

            if(!(frontLeftVal > lineSensorThreshold && frontRightVal > lineSensorThreshold)) {
                currentRoutine = ActionType::DECIDE_CONTROL;
                return;
            }

            return;
        }

        void pickupBox(){
            //picks up the box AND AND AND checks it colour
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
        Bot.checkAllSensorValues(false);
        Bot.checkForNextLocation(); 
        Bot.subRoutine();
    } 
}


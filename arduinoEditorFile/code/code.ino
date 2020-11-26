//this is a test program that should contrain the robot to follow a line.

//THE LOOP AT THE BOTTOM RUNS THE WHOLE PROGRAM
//SET THE CONTROLLER GAINS AT THE START on the line Robot Bot(2,0.5,0.5)
//these can be changed, eg for a pure proportional, choose (2,0,0)
//(proportional, integral, derivative)
#include <Wire.h>

#include <Adafruit_MotorShield.h>

#include "utility/Adafruit_MS_PWMServoDriver.h"

// Include the Servo library 
#include <Servo.h> 

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor * myMotorLeft = AFMS.getMotor(1);
Adafruit_DCMotor * myMotorRight = AFMS.getMotor(2);
// Create a servo object 
Servo Servo1;

bool run = false;

// Declare the Servo pin, 9 for servo2 and 10 for servo1







class Robot {
    public:
        /*ALL PINS */
        int frontLeft = A0; // input pin for FRONT LEFT light sensor
        int frontRight = A1;
        int colourPin = 0;
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
        float distanceFrontVal = 0;
        float distanceVals[NUMBER_OF_SENSOR_POSITIVES] = {0};
        float distanceAvr;
        int colourPinVal = 0;
        

        int lastSensorTriggered = 0;  // 0 for no idea, 1 for left, 2 for right

        /* MOTORS */
        float motorSpeed = 100; //EDIT THIS 
        float lineFollowDampingFactor = 0.9;
        float motorSpeedLeft;
        float motorSpeedRight;
        float speedDifference = 0;
        int pos = 85;  // variable to store the servo position


        /* THRESHOLDS */
        int lineSensorThreshold = 300;
        float distanceSensorThreshold = 460;


        /*DEALING WITH BOXES*/
        bool boxBeingColourChecked = false;
        bool hasBoxAtm = false;
        int pillPosition = 0;
        bool onTargetBox = false;

        enum class ActionType { LINE, TURN_LEFT, TURN_RIGHT, TURN_180, ADVANCE_TO_HOME, CHECK_BOX};
        enum class PositionList { START_BOX,START,FIRST_JUNCTION,TUNNEL,MAIN_T_JUNCTION,PILL,BLUE_TRACK,BLUE_SQUARE };
        enum class Directions {TOWARDS_PILL,AWAY_FROM_PILL};
        enum class BoxCol {RED,BLUE,NO_BOX};

        ActionType currentRoutine = ActionType::LINE;
        PositionList position = PositionList::TUNNEL;//START_BOX;
        Directions direction = Directions::AWAY_FROM_PILL;//TOWARDS_PILL;
        BoxCol currentBoxCol = BoxCol::BLUE;


        void checkForNextLocation(){
            
            static PositionList lastPosition;
            static char place[20] = "start";


            lastPosition = position;
            
            
            if(currentRoutine == ActionType::LINE){  // can only ake changes to postiion when on a line following path so as not tocaue problems with 180 tunrs etc
                switch (position) {


                case PositionList::START_BOX:
                    if(direction == Directions::TOWARDS_PILL){
                        if( farRightVal == 1 || farLeftVal == 1){
                            while ((farLeftVal == 1) || (farRightVal == 1)){
                                checkAllSensorValues(false);
                                binaryFollowLine(100);
                                flashLEDS();
                            }
                            Serial.println("left starting box");
                            position = PositionList::START;
                        }
                    }
                    break;
                case PositionList::START:
                    
                    if((farLeftVal == 1) && (direction == Directions::TOWARDS_PILL)){
                        position = PositionList::FIRST_JUNCTION;
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
                    
                    if(( farRightVal == 1) && (direction == Directions::TOWARDS_PILL) ){
                        position = PositionList::MAIN_T_JUNCTION;
                        strcpy(place,"reached mainJunc");
                    }
                    if(direction == Directions::AWAY_FROM_PILL && farRightVal == 1){
                        position = PositionList::FIRST_JUNCTION;
                    }

                    //check for the sensor positions that would give rise to the next state from here
                    break;
                // case PositionList::MAIN_T_JUNCTION:
                    
                //     //check for the sensor positions that would give rise to the next state from here
                //     if((farLeftVal == 0 || farRightVal == 0) && (direction == Directions::TOWARDS_PILL )){
                //         position = PositionList::PILL;
                //     }
                //     if((farLeftVal == 0 || farRightVal == 0) && (direction == Directions::AWAY_FROM_PILL )){
                //         position = PositionList::TUNNEL;
                //     }
                //     break;
                // case PositionList::PILL:
                //     //if line following can take care of position from here!
                //     break;
                    
                // case PositionList::BLUE_T_JUNCTION:
                //     //check for the sensor positions that would give rise to the next state from here
                //     if((farLeftVal == 0 || farRightVal == 0) && (direction == Directions::AWAY_FROM_PILL)){
                //         position = PositionList::BLUE_BOX_BOTTOM;
                //     }
                //     if((farLeftVal == 0 || farRightVal == 0) && (direction == Directions::TOWARDS_PILL)){
                //         position = PositionList::BLUE_TRACK;
                //     }
                //     break;

                // case PositionList::BLUE_TRACK:
                //     if((direction == Directions::AWAY_FROM_PILL )&&(farLeftVal == 1  && farRightVal == 1)){
                //         position = PositionList::BLUE_T_JUNCTION;
                //     }
                //     if((direction == Directions::TOWARDS_PILL) && (farRightVal == 1)){
                //         position = PositionList::FIRST_JUNCTION;
                //     }
                //     break;
                // case PositionList::BLUE_BOX_BOTTOM:
                //     if(farLeftVal == 1 && direction == Directions::AWAY_FROM_PILL && currentTask == TaskList::PLACE_SECOND_BLUE_BOX){
                //         position = PositionList::BLUE_BOX_BOTTOM_RIGHT;
                //         strcpy(place,"reached blu btm right");
                //     }
                //     if((farLeftVal == 1 || farRightVal == 1) && direction == Directions::TOWARDS_PILL){
                //         position = PositionList::BLUE_T_JUNCTION;
                //         strcpy(place,"reached blu T");
                //     }
                //     break;
                // case PositionList::BLUE_BOX_BOTTOM_RIGHT:
                //     if(farLeftVal == 0 && direction == Directions::AWAY_FROM_PILL){
                //         position = PositionList::BLUE_BOX_RIGHT;
                //         strcpy(place,"reached blu right side");
                //     }
                //     if(farRightVal == 0 && direction == Directions::TOWARDS_PILL){
                //         position = PositionList::BLUE_BOX_BOTTOM;
                //         strcpy(place,"reached blu btm side");
                //     }
                //     break;
                // case PositionList::BLUE_BOX_RIGHT:
                //     if(farLeftVal == 1 && direction == Directions::AWAY_FROM_PILL){
                //         position = PositionList::BLUE_BOX_TOP_RIGHT;
                //         strcpy(place,"reached blu top right");
                //     }
                //     if(farRightVal == 1 && direction == Directions::TOWARDS_PILL){
                //         position = PositionList::BLUE_BOX_BOTTOM_RIGHT;
                //         strcpy(place,"reached blu btm right");
                //     }
                //     break;
                // case PositionList::BLUE_BOX_TOP_RIGHT:
                //     if(farLeftVal == 0 && direction == Directions::AWAY_FROM_PILL){
                //         position = PositionList::BLUE_BOX_TOP;
                //         strcpy(place,"reached blu box top");
                //     }
                //     if(farRightVal == 0 && direction == Directions::TOWARDS_PILL){
                //         position = PositionList::BLUE_BOX_RIGHT;
                //         strcpy(place,"reached blu right side");
                //     }
                //     break;
                // case PositionList::BLUE_BOX_TOP:
                //     if(farRightVal == 1 && direction == Directions::TOWARDS_PILL){
                //         position = PositionList::BLUE_BOX_TOP_RIGHT;
                //         strcpy(place,"reached blu top right");
                //     }
                //     break;
                }

                if(position != lastPosition){
                Serial.println(place);
            }
            }
        }

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
            float distanceSum = 0;
            
            for(int i = 1; i<NUMBER_OF_SENSOR_POSITIVES -1 ; i++ ){
                rightVals[i] = rightVals[i+1];
                leftVals[i] = leftVals[i+1];
                middleVals[i] = middleVals[i+1];
                distanceVals[i] = distanceVals[i+1];

                rightSum += rightVals[i];
                leftSum +=leftVals[i];
                middleSum +=middleVals[i];
                distanceSum += distanceVals[i];
            }
            leftVals[NUMBER_OF_SENSOR_POSITIVES -1] = digitalRead(offAxisLeft) ;
            rightVals[NUMBER_OF_SENSOR_POSITIVES -1] = digitalRead(offAxisRight) ;
            middleVals[NUMBER_OF_SENSOR_POSITIVES -1] = digitalRead(backMiddle) ;
            distanceVals[NUMBER_OF_SENSOR_POSITIVES -1] = analogRead(distanceSensor); 

            leftSum += leftVals[NUMBER_OF_SENSOR_POSITIVES -1];
            rightSum +=rightVals[NUMBER_OF_SENSOR_POSITIVES -1];
            middleSum +=middleVals[NUMBER_OF_SENSOR_POSITIVES -1];
            distanceSum += distanceVals[NUMBER_OF_SENSOR_POSITIVES -1];

            farLeftVal = leftSum > 8 ? 1: 0; 
            farRightVal = rightSum > 8 ? 1: 0; 
            backMiddleVal = middleSum > 8 ? 1: 0; 
            distanceFrontVal = distanceSum / NUMBER_OF_SENSOR_POSITIVES;

            if(listVals){
                // Serial.print("front left val:  ");
                // Serial.println(frontLeftVal);
                // Serial.print("front right val: ");
                // Serial.println(frontRightVal);

                // Serial.print("far right val:");
                // Serial.println(farRightVal);
                // Serial.print("far left val:");
                // Serial.println(farLeftVal);
                // Serial.print("back middle Val: ");
                // Serial.println(backMiddleVal);
                // Serial.print("distanceSensor: ");
                // Serial.println(distanceFrontVal);
                // Serial.println("                                     ");
                // Serial.println("                                     ");
            }


            colourPinVal = digitalRead(colourPin);
        }

        void binaryFollowLine(int increaseRate) { 
            //COULD WE USE PROPORTIONAL CONTROL FOR THIS IE SPEED DIFFERENCE IS PROPORTIONAL TO LINESENSOR READING (POTENTIALLY ONLY IF ITS ABOVE THRESHOLD)
            //THIS WILL ALLOW US TO BE REALLY STRAIGHT ON THE STRAGHT BITS AND SO BLOCK PLACEMENT WILL BECOME SIMPLER...
            // if(frontLeftVal > lineSensorThreshold && frontRightVal > lineSensorThreshold){
            //     while (frontLeftVal > lineSensorThreshold && frontRightVal > lineSensorThreshold){
            //         /*
            //         if(lastSensorTriggered == 2){ 
            //             runMotors(-1*motorSpeed,1*motorSpeed);
            //         } else if (lastSensorTriggered == 1) {
            //             runMotors(1*motorSpeed,-1*motorSpeed);
            //         }
            //         */
            //         runMotors(50,50);
            //         checkAllSensorValues(false);
            //     }
            // }
            if(frontLeftVal > lineSensorThreshold && frontRightVal > lineSensorThreshold){
                runMotors(motorSpeed,motorSpeed);
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
            
            // while(farLeftVal == 1 ){
            //     checkAllSensorValues(false);
            //     runMotors(-1*motorSpeed,1*motorSpeed);
            // }

            // while (farLeftVal == 0 ) {
            //     runMotors(-1*motorSpeed,1*motorSpeed); 
            //     checkAllSensorValues(false);
            // }
            if(frontLeftVal > lineSensorThreshold ){
                while (frontLeftVal > lineSensorThreshold){
                    checkAllSensorValues(false);
                    runMotors(-1*motorSpeed,1*motorSpeed);
                }
            }
            while(frontLeftVal < lineSensorThreshold ){
                checkAllSensorValues(false);
                runMotors(-1*motorSpeed,1*motorSpeed);
            }
            while(frontRightVal < lineSensorThreshold ){
                checkAllSensorValues(false);
                runMotors(-1*motorSpeed,1*motorSpeed);
            }
            int timer = 0;
            while (timer < 100){
                timer +=1;
                binaryFollowLine(100);
                checkAllSensorValues(false);
                flashLEDS();
            }
            return;
        }

        void turnRight() {
            //WAIT FOR FAR LEFT TO TRIGGER
            
            // while(farLeftVal == 1 ){
            //     checkAllSensorValues(false);
            //     runMotors(-1*motorSpeed,1*motorSpeed);
            // }

            // while (farLeftVal == 0 ) {
            //     runMotors(-1*motorSpeed,1*motorSpeed); 
            //     checkAllSensorValues(false);
            // }
            if(frontRightVal > lineSensorThreshold ){
                while (frontRightVal > lineSensorThreshold){
                    checkAllSensorValues(false);
                    runMotors(1*motorSpeed,-1*motorSpeed);
                }
            }
            while(frontRightVal < lineSensorThreshold ){
                checkAllSensorValues(false);
                runMotors(1*motorSpeed,-1*motorSpeed);
            }
            while(frontLeftVal < lineSensorThreshold ){
                checkAllSensorValues(false);
                runMotors(1*motorSpeed,-1*motorSpeed);
            }
            int timer = 0;
            while (timer < 100){
                timer +=1;
                binaryFollowLine(100);
                checkAllSensorValues(false);
                flashLEDS();
            }
            return;
        }

        void turn180() {
            //WAIT FOR FAR LEFT TO TRIGGER
            
            while(backMiddle == 1 ){
                checkAllSensorValues(false);
                runMotors(1*motorSpeed,-1*motorSpeed);
            }

            // while (farLeftVal == 0 ) {
            //     runMotors(-1*motorSpeed,1*motorSpeed); 
            //     checkAllSensorValues(false);
            // }
            while( farRightVal == 0 ){
                checkAllSensorValues(false);
                runMotors(1*motorSpeed,-1*motorSpeed);
            }
            while(frontRightVal < lineSensorThreshold ){
                checkAllSensorValues(false);
                runMotors(1*motorSpeed,-1*motorSpeed);
            }
            while(frontLeftVal < lineSensorThreshold ){
                checkAllSensorValues(false);
                runMotors(1*motorSpeed,-1*motorSpeed);
            }
            return;

        }

        void flashLEDS(){
            static int timer = 0;
            int state = LOW;
            timer += 1;
            if(timer > 100) timer = 1;
            if(timer % 100 == 0 ){
                state = (state == HIGH) ? LOW : HIGH;
            }

            digitalWrite(ledPinSecond,state);



            switch (currentBoxCol){
                case BoxCol::BLUE:
                    digitalWrite(ledPinThird,HIGH);
                    digitalWrite(ledPinFirst,LOW);

                    break;
                case BoxCol::RED:
                    digitalWrite(ledPinThird,LOW);
                    digitalWrite(ledPinFirst,HIGH);
                    break;
                case BoxCol::NO_BOX:
                    digitalWrite(ledPinThird,LOW);
                    digitalWrite(ledPinFirst,LOW);
                    break;
            }
        }


        void stopInHomeLocation(){
            int timer = 0;
            while (timer < 1000){
                timer += 1;
                runMotors(motorSpeed,motorSpeed);
            }
            Serial.println("stopping NOWWWW");
            runMotors(0,0);
            run = false;
        }

        void chooseAction(){
            currentRoutine = ActionType::LINE;
            

            if(farRightVal == 1 && position == PositionList::PILL && direction == Directions::AWAY_FROM_PILL ){
                currentRoutine = ActionType::TURN_RIGHT;
                pillPosition = 0;
                Serial.println("reached main t junction, coming home");
                position = PositionList::TUNNEL;
            } 
            if(position == PositionList::PILL && !boxBeingColourChecked && !hasBoxAtm){
                if(distanceFrontVal > 500){
                    Serial.println("checking box colour");
                    currentRoutine = ActionType::CHECK_BOX;
                    boxBeingColourChecked = true;
                }

                if(farLeftVal == 1 && farRightVal == 1 && onTargetBox == false){
                    Serial.println("ran over target box");

                    pillPosition ++;
                    onTargetBox = true;

                }
                if(farLeftVal == 0 && farRightVal == 0 && onTargetBox == true){
                    onTargetBox = false;
                }


            }

            if(position == PositionList::MAIN_T_JUNCTION && direction == Directions:: TOWARDS_PILL ){
                currentRoutine = ActionType::TURN_LEFT;
                Serial.println("at pill");
                position = PositionList::PILL;
            }



            if(position == PositionList::FIRST_JUNCTION && direction == Directions::AWAY_FROM_PILL){

                if(currentBoxCol == BoxCol::NO_BOX){
                    Serial.println("at first junction for the lsast time");

                    while (frontRightVal<lineSensorThreshold){

                        checkAllSensorValues(false);
                        runMotors(motorSpeed-60,motorSpeed+60);
                        flashLEDS();
                    }
                    position = PositionList::START;
                }
                if(currentBoxCol == BoxCol::BLUE){
                    turnRight();
                    position = PositionList::BLUE_TRACK;
                    Serial.println("on blue track");

                    
                }

                currentRoutine = ActionType::LINE;

            } 
            if(position == PositionList::START && direction == Directions::AWAY_FROM_PILL){
                currentRoutine = ActionType::LINE;
                if(farLeftVal == 1 || farRightVal == 1){
                    Serial.println("on starting track, about to stop");
                    stopInHomeLocation();
                }
                
            }

            if(position == PositionList::BLUE_TRACK && direction == Directions::AWAY_FROM_PILL && (farLeftVal == 1 || farRightVal ==1) ){
                currentRoutine = ActionType::TURN_LEFT;
                Serial.println("at blue square");
                position = PositionList::BLUE_SQUARE;
            }

            runAction();
        }

        void checkBoxColour(){
            static char colour[20];
            strcpy(colour,"blue");
            currentBoxCol = BoxCol::BLUE;
            while(distanceFrontVal > 350 ){
                runMotors(motorSpeed,motorSpeed);
                checkAllSensorValues(false);
                if(colourPinVal == 1){
                    strcpy(colour,"red");
                    currentBoxCol = BoxCol::RED;
                }
            }
            flashLEDS();
            Serial.println(colour);
            //servo code

            // for (pos = 80; pos <= 150; pos += 1) { // goes from 0 degrees to 180 degrees
            //     // in steps of 1 degree
            //     Servo1.write(pos);              // tell servo to go to position in variable 'pos'
            //     delay(40);                       // waits 15ms for the servo to reach the position
            // }




            //servo code end

            static int timer = 0;
            while (timer < 600){
                timer +=1 ;
                runMotors(-1*motorSpeed,-1*motorSpeed);
            }
            turn180();
            direction = Directions::AWAY_FROM_PILL;
            boxBeingColourChecked = false;
            hasBoxAtm = true;
        }

        void runAction(){
            switch(currentRoutine){
                case ActionType::LINE:
                    binaryFollowLine(100);
                    break;
                case ActionType::TURN_LEFT:
                    turnLeft();
                    break;
                case ActionType::TURN_RIGHT:
                    turnRight();
                    break;
                case ActionType::TURN_180:
                    turn180();
                    break;
                case ActionType::ADVANCE_TO_HOME:
                    stopInHomeLocation();
                    run = false;
                    break;
                case ActionType::CHECK_BOX:
                    checkBoxColour();
                    break;
            }
        }
};

Robot Bot;

void setup() {

    AFMS.begin();
    Serial.begin(9600);
    Servo1.attach(10);

    pinMode(Bot.startButtonPin, INPUT);
    pinMode(Bot.offAxisLeft,INPUT);
    pinMode(Bot.offAxisRight,INPUT);

    pinMode(Bot.ledPinFirst,OUTPUT);
    pinMode(Bot.ledPinSecond,OUTPUT);
    pinMode(Bot.ledPinThird,OUTPUT);

    // Servo1.write(90);
    // delay(10);
}

void loop() {
    Bot.OnOffSwitch();
    
    
    if(run == true){
        Bot.flashLEDS();
        Bot.checkAllSensorValues(true);
        Bot.checkForNextLocation();
        Bot.chooseAction();
    } else {
        Bot.runMotors(0,0);
    }
}

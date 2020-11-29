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
        bool clockwise = true;

        enum class ActionType { LINE, TURN_LEFT, TURN_RIGHT, TURN_180, ADVANCE_TO_HOME, CHECK_BOX};
        enum class PositionList { START_BOX,START,FIRST_JUNCTION,TUNNEL,MAIN_T_JUNCTION,PILL,BLUE_TRACK,BLUE_T,BLUE_CORNER, BLUE_SIDE };
        enum class Directions {TOWARDS_PILL,AWAY_FROM_PILL};
        enum class BoxCol {RED,BLUE,NO_BOX};

        ActionType currentRoutine = ActionType::LINE;
        PositionList position = PositionList::TUNNEL;//START_BOX;
        Directions direction = Directions::AWAY_FROM_PILL;//TOWARDS_PILL;
        BoxCol currentBoxCol = BoxCol::BLUE;

        //the whole contorl structre
            /*line follow to t junction, the turn left
            if red 
                check other side ()  -> go back round until you hit the t-junction, then carry on until you hit something on the other side
                if red 
                    put it on the left hand side ()  -> pickup a red, 180, cross t- junction, carry on for a short while then place on the left hand side 
                    then go back right, pickup and place blue,
                    come back, head right and pickup and place blue

                    come back and place the left red in the right hand spot, 
                    then go back and plave the last left red in hte clockwise spot
                    then go home
                else if blue 
                    place the blue and check the right hand side again
                    if red:
                        move to left hand side 
                        then come back, go right pick up and place the last blue
                        and then place the left red on the right
                        then place the last red clockwise
                    else 
                        place the blue,
                        come back and place the left red on the right
                        go across and place the left red clockwise on the pill
            else pickup and place blue
                come back and check the left hand side again 
                if red:
                    check other side
                    if red: 
                        place this red on left
                        go back and deal with final blue on right
                        go back and place the first lefthandside red on the right hand side
                        then deal with second red on the left hand side
                else blue:
                    place blue and 
                    place the right red on the right
                    place the clockwise red on the back of the pill.
            */

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

        void OnOffSwitch() {
            //sets start program to true at the push of the button
            int buttonState = digitalRead(startButtonPin);
            static bool lockSwitch = false;

            if (buttonState == 0 && lockSwitch==false) {
                run = false;
                lockSwitch = true;
                runMotors(0,0);
                digitalWrite(ledPinFirst, LOW);
                digitalWrite(ledPinSecond, LOW);
                digitalWrite(ledPinThird,LOW);
                Serial.println("Pausing Program");
                while(run == false){
                    int buttonState = digitalRead(startButtonPin);
                    if (buttonState == 1 && lockSwitch ==true) {
                        lockSwitch = false;
                    }
                    if(buttonState == 0 && lockSwitch == false){
                        run = true;
                        lockSwitch = true;
                        return;
                    }
                }
            }
            if (buttonState == 1 && lockSwitch == true) {
                lockSwitch = false;
            }
        }
        
        void utilityFunction(){
            checkAllSensorValues(false);
            flashLEDS();
            OnOffSwitch();
            return;
        }

        void follow(int loops){
            int timer = 0;
            while (timer < loops){
                timer +=1;
                binaryFollowLine(100);
                checkAllSensorValues(false);
                flashLEDS();
            }
            return;
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

        void binaryFollowLine(int increaseRate) {
            if(position == PositionList::PILL){
               if(farLeftVal == 1 && farRightVal == 1 && onTargetBox == false){
                    if(clockwise==true){
                        pillPosition ++;
                        Serial.println("ran over clockwise target box");
                    } else{
                        pillPosition --;
                        Serial.println("ran over anticlockwise target box");
                    }
                    onTargetBox = true;
                }
                if(farLeftVal == 0 && farRightVal == 0 && onTargetBox == true){
                    onTargetBox = false;
                }
                if(frontLeftVal > lineSensorThreshold && frontRightVal > lineSensorThreshold){
                    runMotors(motorSpeed,motorSpeed);
                } 
            }
            if(position == PositionList::BLUE_SIDE){
                if(farLeftVal == 1 && farRightVal == 1 && onTargetBox == false){
                    onTargetBox = true;
                }
                if(farLeftVal == 0 && farRightVal == 0 && onTargetBox == true){
                    onTargetBox = false;
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
            follow(100);
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


        void loop() {
            while(!(position == PositionList::START)){
                utilityFunction();
                binaryFollowLine(100);
                if(farLeftVal == 1 && farRightVal == 1){
                    position = PositionList::START;
                }
            }
            while(!(position == PositionList::FIRST_JUNCTION)){
                utilityFunction();
                binaryFollowLine(100);
                if( farLeftVal == 1){
                    position = PositionList::FIRST_JUNCTION;
                }
            }
            while(!(position == PositionList::TUNNEL)){
                utilityFunction();
                binaryFollowLine(100);
                if( farLeftVal == 0){
                    position = PositionList::TUNNEL;
                }
            }
            while(!(position == PositionList::MAIN_T_JUNCTION)){
                utilityFunction();
                binaryFollowLine(100);
                if(farLeftVal == 1 && farRightVal == 1){
                    position = PositionList::MAIN_T_JUNCTION;
                }
            }
            while(!(position == PositionList::PILL)){
                utilityFunction();
                turnLeft();
                position = PositionList::PILL;
            }
            while(!(distanceFrontVal > 500)){
                utilityFunction();
                binaryFollowLine(100);
            }
            while(!(currentBoxCol == BoxCol::NO_BOX)){
                utilityFunction();
                checkBoxColour();
            }
            if(currentBoxCol == BoxCol::BLUE){
                //first blue box on clockwise side
                while(!(hasBoxAtm==true)){
                    utilityFunction();
                    pickupBox();
                }
                while(!(clockwise==false)){
                    utilityFunction();
                    turn180();
                    clockwise = false;
                }
                while(!(pillPosition== 0)){
                    utilityFunction();
                    binaryFollowLine(100);
                }
                while(!(position == PositionList::MAIN_T_JUNCTION)){
                    utilityFunction();
                    binaryFollowLine(100);
                    if (farRightVal==1){
                        position == PositionList::MAIN_T_JUNCTION;
                    }
                }
                while(!(position == PositionList::TUNNEL)){
                    utilityFunction();
                    turnRight();
                    direction = Directions::AWAY_FROM_PILL;
                    position = PositionList::TUNNEL;
                }
                placeFirstBlueBox();
                
                while(!(position == PositionList::PILL)){
                    utilityFunction();
                    turnLeft();
                    position = PositionList::PILL;
                    clockwise = true;
                }
                while(!(distanceFrontVal > 500)){
                    utilityFunction();
                    binaryFollowLine(100);
                }
                while(!(currentBoxCol == BoxCol::NO_BOX)){
                    utilityFunction();
                    checkBoxColour();
                }
                if(currentBoxCol == BoxCol::BLUE){
                    //second blue box on clockwise side
                    while(!(hasBoxAtm==true)){
                        utilityFunction();
                        pickupBox();
                    }
                    while(!(clockwise==false)){
                        utilityFunction();
                        turn180();
                        clockwise = false;
                    }
                    while(!(pillPosition== 0)){
                        utilityFunction();
                        binaryFollowLine(100);
                    }
                    while(!(position == PositionList::MAIN_T_JUNCTION)){
                        utilityFunction();
                        binaryFollowLine(100);
                        if (farRightVal==1){
                            position == PositionList::MAIN_T_JUNCTION;
                        }
                    }
                    while(!(position == PositionList::TUNNEL)){
                        utilityFunction();
                        turnRight();
                        direction = Directions::AWAY_FROM_PILL;
                        position = PositionList::TUNNEL;
                    }
                    placeSecondBlueBox();
                }


            }
        }

        void placeFirstBlueBox(){
            //goes from tunel back to main junction
            while(!(position == PositionList::FIRST_JUNCTION)){
                utilityFunction();
                binaryFollowLine(100);
                if(farRightVal == 1 ){
                    position == PositionList::FIRST_JUNCTION;
                }
            }
            while(!(position == PositionList::BLUE_TRACK)){
                utilityFunction();
                turnRight();
                position == PositionList::BLUE_TRACK;
            }
            while(!(position == PositionList::BLUE_T)){
                utilityFunction();
                binaryFollowLine(100);
                if(farRightVal == 1 && farLeftVal == 1){
                    position = PositionList::BLUE_T;
                }
            }
            while(!(position == PositionList::BLUE_SIDE)){
                utilityFunction();
                turnLeft();
                position = PositionList::BLUE_SIDE;
            }
            while(!(onTargetBox==true)){
                utilityFunction();
                binaryFollowLine(100);
            }
            while(!(direction == Directions::TOWARDS_PILL)){
                utilityFunction();
                placeBox();
                direction = Directions::TOWARDS_PILL;
            }
            while(!(position == PositionList::BLUE_T)){
                utilityFunction();
                binaryFollowLine(100);
                if(farRightVal==1){
                    position = PositionList::BLUE_T;
                }
            }
            while(!(position == PositionList::BLUE_TRACK)){
                utilityFunction();
                turnRight();
                position =  PositionList::BLUE_TRACK;
            }
            while(!(position == PositionList::FIRST_JUNCTION)){
                utilityFunction();
                binaryFollowLine(100);
                if(farRightVal == 1){
                    position = PositionList::FIRST_JUNCTION;
                }
            }
            while(!(position == PositionList::MAIN_T_JUNCTION)){
                utilityFunction();
                binaryFollowLine(100);
                if(farRightVal ==1 && farLeftVal ==1){
                    position = PositionList::MAIN_T_JUNCTION;
                }
            }
        }

        void placeSecondBlueBox(){
            while(!(position == PositionList::FIRST_JUNCTION)){
                utilityFunction();
                binaryFollowLine(100);
                if(farRightVal == 1 ){
                    position == PositionList::FIRST_JUNCTION;
                }
            }
            while(!(position == PositionList::BLUE_TRACK)){
                utilityFunction();
                turnRight();
                position == PositionList::BLUE_TRACK;
            }
            while(!(position == PositionList::BLUE_T)){
                utilityFunction();
                binaryFollowLine(100);
                if(farRightVal == 1 && farLeftVal == 1){
                    position = PositionList::BLUE_T;
                }
            }
            //jumps across to other side of the square
            int timer = 0;
            runMotors(motorSpeed,motorSpeed);
            while (timer < 200){
                timer +=1;
                utilityFunction();
            }
            while(!(position == PositionList::BLUE_SIDE)){
                utilityFunction();
                binaryFollowLine(100);
                if(farRightVal==1 && farLeftVal ==1){
                    turnLeft();
                    position = PositionList::BLUE_SIDE;
                }
            }
            while(!(onTargetBox==true)){
                utilityFunction();
                binaryFollowLine(100);
            }
            while(!(direction == Directions::TOWARDS_PILL)){
                utilityFunction();
                placeBox();
                direction = Directions::TOWARDS_PILL;
            }
            while(!(position == PositionList::BLUE_CORNER)){
                utilityFunction();
                binaryFollowLine(100);
                if(farRightVal ==1){
                    position = PositionList::BLUE_CORNER;
                }
            }
            while(!(position == PositionList::BLUE_SIDE)){
                utilityFunction();
                turnRight();
                position = PositionList::BLUE_SIDE
            }
            while(!(position == PositionList::BLUE_CORNER)){
                utilityFunction();
                binaryFollowLine(100);
                if(farRightVal ==1){
                    position = PositionList::BLUE_CORNER;
                }
            }
            while(!(position == PositionList::BLUE_SIDE)){
                utilityFunction();
                turnRight();
                position = PositionList::BLUE_SIDE
            }
            while(!(position == PositionList::BLUE_T)){
                utilityFunction();
                binaryFollowLine(100);
                if(farLeftVal ==1){
                    position = PositionList::BLUE_T
                }
            }
            while(!(position == PositionList::BLUE_TRACK)){
                utilityFunction();
                turnLeft();
                position = PositionList::BLUE_TRACK;
            }
            while(!(position == PositionList::FIRST_JUNCTION)){
                utilityFunction();
                binaryFollowLine(100);
                if(farRightVal == 1){
                    position = PositionList::FIRST_JUNCTION;
                }
            }
            while(!(position == PositionList::MAIN_T_JUNCTION)){
                utilityFunction();
                binaryFollowLine(100);
                if(farRightVal ==1 && farLeftVal ==1){
                    position = PositionList::MAIN_T_JUNCTION;
                }
            }
        }


        void pickupBox(){
            //closes claws and sets hasBoxatm to true
        }

        void placeBox(){
            /*if onTargetLocation == true{
                reverse a bit
                diposit box
                reverse some more
                set on Targetlocation to zero 
                set currentBox col to no Box
                do a 180
            }*/
        }

        void checkForNextLocation(){
            
            static PositionList lastPosition;
            static char place[20] = "start";


            lastPosition = position;
            
            
            if(currentRoutine == ActionType::LINE){  // can only ake changes to postiion when on a line following path so as not tocaue problems with 180 tunrs etc
                switch (position) {
                case PositionList::START_BOX:
                    if(direction == Directions::TOWARDS_PILL){
                        if(farRightVal == 1 || farLeftVal == 1){
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
            
                }

                if(position != lastPosition){
                Serial.println(place);
            }
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
            boxBeingColourChecked = false;
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
                    turn180();
                    direction = Directions::AWAY_FROM_PILL;
                    hasBoxAtm = true;
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



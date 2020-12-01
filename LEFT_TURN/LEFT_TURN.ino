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
int servoStart = 60;
int servoClose = 85;

// Declare the Servo pin, 9 for servo2 and 10 for servo1







class Robot {
    public:
        /*ALL PINS */
        int frontLeft = A0; // input pin for FRONT LEFT light sensor
        int frontRight = A1;
        int colourPin = 0;
        int offAxisRight = 4;
        int offAxisLeft = 3;
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
        int pillPosition = 0;
        bool onTargetBox = false;
        bool clockwise = true;
        bool hasBoxAtm = false;

        enum class PositionList { START_BOX,START,FIRST_JUNCTION,TUNNEL,MAIN_T_JUNCTION,PILL,BLUE_TRACK,BLUE_T,BLUE_CORNER, BLUE_SIDE };
        enum class Directions {TOWARDS_PILL,AWAY_FROM_PILL};
        enum class BoxCol {RED,BLUE,NO_BOX};

        PositionList position = PositionList::START_BOX;//START_BOX;
        Directions direction = Directions::TOWARDS_PILL;//TOWARDS_PILL;
        BoxCol currentBoxCol = BoxCol::NO_BOX;

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

        void onOffSwitch() {
            //sets start program to true at the push of the button
            int buttonState = digitalRead(startButtonPin);
            static bool lockSwitch = false;

            if(run == false){
                while(buttonState == 1 && run == false){
                    int buttonState = digitalRead(startButtonPin);
                    if(buttonState == 0){
                        Serial.println("Begin Program");
                        run = true;
                        lockSwitch = true;
                        return;
                    }
                }
            }
            
            if(buttonState == 0 && lockSwitch == false && run == true){
                Serial.println("Pause Program");
                run = false;
                runMotors(0,0);
                digitalWrite(ledPinFirst, LOW);
                digitalWrite(ledPinSecond, LOW);
                digitalWrite(ledPinThird,LOW);
                lockSwitch = true;
                while(run == false){
                    int buttonState = digitalRead(startButtonPin);
                    if (buttonState == 1 && lockSwitch ==true) {
                        lockSwitch = false;
                    }
                    if(buttonState == 0 && lockSwitch == false){
                        Serial.println("Continue Program");
                        run = true;
                        lockSwitch = true;
                        return;
                    }
                }
            }
            if(buttonState == 1 && lockSwitch == true){
                lockSwitch = false;
            }
        }
        
        void utilityFunction(){
            checkAllSensorValues(false);
            flashLEDS();
            onOffSwitch();
            return;
        }

        void follow(int loops){
            int timer = 0;
            while (timer < loops){
                timer +=1;
                binaryFollowLine(100);
                utilityFunction();
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
            checkAllSensorValues(false);
            onOffSwitch();
            if (frontLeftVal > lineSensorThreshold) {
                speedDifference = increaseRate;
            }
            else if (frontRightVal > lineSensorThreshold) {
                speedDifference = -1 * increaseRate;
            }
            
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
                if(onTargetBox == true){
                    runMotors(motorSpeed,motorSpeed);
                } 
            }

            if(position == PositionList::BLUE_SIDE){
                if(farLeftVal == 1 && farRightVal == 1 && onTargetBox == false){
                    onTargetBox = true;
                    Serial.println("On Blue Target Box");
                }
                if(farLeftVal == 0 && farRightVal == 0 && onTargetBox == true){
                    onTargetBox = false;
                }
            }

            speedDifference *= lineFollowDampingFactor;
            motorSpeedLeft = motorSpeed - speedDifference;
            motorSpeedRight = motorSpeed + speedDifference;
            runMotors(motorSpeedLeft,motorSpeedRight);
            return;
        }

        void turnLeft() {
            Serial.println("Turning Left");
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
            Serial.println("Turning Right");
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
            follow(100);
            return;
        }

        void turn180() {
            Serial.println("Turning 180");
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

        //used either having just deposited or decided against picking up, tune so that it just misses block on 180
        void reverseAndTwist(){
            int timer = 0;
            Serial.println("reversing");
            while (timer < 700){
                timer +=1;
                checkAllSensorValues(false);
                flashLEDS();
                runMotors(-1*motorSpeed,-1*motorSpeed);
            }
            Serial.println("finished reversing");
            turn180();
        }

        void pickupBox(){
            //closes claws and sets hasBoxatm to true
            hasBoxAtm = true;
            for (pos = servoStart; pos <= servoClose; pos += 1) { // goes from 0 degrees to 180 degrees
                // in steps of 1 degree
                Servo1.write(pos);              // tell servo to go to position in variable 'pos'
                delay(50);                       // waits 15ms for the servo to reach the position
            }            
        }

        void placeBox(){
            /*
                reverse a bit
                diposit box
                reverse some more
                set onTargetlocation to zero 
                set currentBox col to no Box
                SET HASBOXATM TO FALSE
                do a 180
            */
            Serial.println("placing box");
            int timer = 0;
            runMotors(-1*motorSpeed,-1*motorSpeed);
            while (timer < 400){
                timer +=1;
                checkAllSensorValues(false);
                flashLEDS();
            }
            runMotors(0,0);
            for (pos = servoClose; pos >= servoStart; pos -= 1) { // goes from 0 degrees to 180 degrees
                // in steps of 1 degree
                Servo1.write(pos);              // tell servo to go to position in variable 'pos'
                delay(50);                       // waits 15ms for the servo to reach the position
            }   
            timer = 0;
            runMotors(-1*motorSpeed,-1*motorSpeed);
            while (timer < 200){
                timer +=1;
                checkAllSensorValues(false);
                flashLEDS();
            }     
            turn180();
            onTargetBox = false;
            currentBoxCol = BoxCol::NO_BOX;
            hasBoxAtm = false;


            // GET CLAW TO OPEN;
            // IF BOX HAS SLIPPED TO END, WILL NOT BE ABLE TO REVERSE AND TWIST
        }

        //currently not using stop in home location
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
            Serial.println("Checking Box Colour");
            static char colour[20];
            strcpy(colour,"blue");
            currentBoxCol = BoxCol::BLUE;
            while(distanceFrontVal > 350 ){
                checkAllSensorValues(false);
                binaryFollowLine(100);
                if(colourPinVal == 1){
                    strcpy(colour,"red");
                    currentBoxCol = BoxCol::RED;
                }
            }
            runMotors(0,0);
            utilityFunction();
            runMotors(0,0);
            Serial.println(colour);
        }


        //places first blue box, starts from tunnel and ends on T-junction
        void placeSecondBlueBox(){
            // Serial.println("Placing First Blue Block");
            // // goes from tunel back to main junction
            while(!(position == PositionList::FIRST_JUNCTION)){
                utilityFunction();
                binaryFollowLine(100);
                Serial.println(farRightVal);
                if(farRightVal == 1 ){
                    position = PositionList::FIRST_JUNCTION;
                }
            }
            while(!(position == PositionList::BLUE_TRACK)){
                Serial.println("Reached First Junction");
                utilityFunction();
                turnRight();
                position = PositionList::BLUE_TRACK;
            }
            while(!(position == PositionList::BLUE_T)){
                utilityFunction();
                binaryFollowLine(100);
                if(farRightVal == 1 && farLeftVal == 1){
                    position = PositionList::BLUE_T;
                }
            }
            Serial.println("reached blue T");
            while(!(position == PositionList::BLUE_SIDE)){
                utilityFunction();
                lineSensorThreshold = 200;
                turnLeft();
                position = PositionList::BLUE_SIDE;
            }
            direction = Directions::AWAY_FROM_PILL;
            while(!(direction == Directions::TOWARDS_PILL)){
                utilityFunction();
                int timer = 0;
                while (timer < 20){
                    timer +=1;
                    runMotors(motorSpeed,motorSpeed);
                    utilityFunction();
                }
                hasBoxAtm = true;
                for (pos = servoClose; pos > servoStart; pos -= 1) { // 
                    // in steps of 1 degree
                    Servo1.write(pos);              // tell servo to go to position in variable 'pos'
                    delay(50);                       // waits 15ms for the servo to reach the position
                }
                Serial.println("Opened Claws");
                timer = 0;
                while (timer < 1000){
                    timer +=1;
                    runMotors(-motorSpeed,-motorSpeed);
                    utilityFunction();
                }
                direction = Directions::TOWARDS_PILL;
            }
            Serial.println("finished manual seqence");

            while(!(position == PositionList::BLUE_T)){
                utilityFunction();
                binaryFollowLine(100);
                if(farLeftVal==1){
                    position = PositionList::BLUE_T;
                }
            }
            Serial.println("Reached Blue T just placed");
            while(!(position == PositionList::BLUE_TRACK)){
                utilityFunction();
                turnLeft();
                lineSensorThreshold = 300;
                position =  PositionList::BLUE_TRACK;
            }
            Serial.println("Reached Blue Track");
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

        void placeFirstBlueBox(){
            Serial.println("Placing Second Blue Block");
            // while(!(position == PositionList::FIRST_JUNCTION)){
            //     utilityFunction();
            //     binaryFollowLine(100);
            //     if(farRightVal == 1 ){
            //         position == PositionList::FIRST_JUNCTION;
            //     }
            // }
            // while(!(position == PositionList::BLUE_TRACK)){
            //     utilityFunction();
            //     turnRight();
            //     position == PositionList::BLUE_TRACK;
            // }
            while(!(position == PositionList::BLUE_T)){
                utilityFunction();
                binaryFollowLine(100);
                if(farRightVal == 1 && farLeftVal == 1){
                    position = PositionList::BLUE_T;
                }
            }
            //jumps across to other side of the square
            while (frontLeftVal < lineSensorThreshold && frontRightVal < lineSensorThreshold){
                runMotors(motorSpeed,motorSpeed);
                utilityFunction();
            }
            int timer = 0;
            while(timer < 250){
                runMotors(1*motorSpeed,1*motorSpeed);
                timer +=1;
            }
            while(!(position == PositionList::BLUE_SIDE)){
                utilityFunction();
                while(frontLeftVal < lineSensorThreshold){
                    runMotors(-1*motorSpeed,motorSpeed);
                    utilityFunction();
                }
                while(frontRightVal < lineSensorThreshold){
                    runMotors(-1*motorSpeed,motorSpeed);
                    utilityFunction();
                }
                position = PositionList::BLUE_SIDE;
            }
            direction = Directions::AWAY_FROM_PILL;
            while(!(direction == Directions::TOWARDS_PILL)){
                utilityFunction();
                int timer = 0;
                while (timer < 20){
                    timer +=1;
                    runMotors(motorSpeed,motorSpeed);
                    utilityFunction();
                }
                hasBoxAtm = true;
                for (pos = servoClose; pos > servoStart; pos -= 1) { // 
                    // in steps of 1 degree
                    Servo1.write(pos);              // tell servo to go to position in variable 'pos'
                    delay(50);                       // waits 15ms for the servo to reach the position
                }
                Serial.println("Opened Claws");
                timer = 0;
                while (timer < 1000){
                    timer +=1;
                    runMotors(-motorSpeed,-motorSpeed);
                    utilityFunction();
                }
                timer = 0;
                while (timer < 300){
                    timer +=1;
                    runMotors(-motorSpeed,motorSpeed);
                    utilityFunction();
                }
                runMotors(motorSpeed,motorSpeed);
                direction = Directions::TOWARDS_PILL;
            }
            Serial.println("finished manual seqence");
            
            while(!(farRightVal)){
                utilityFunction();
                position = PositionList::BLUE_SIDE;
            }
            while(!(position == PositionList::BLUE_T)){
                utilityFunction();
                binaryFollowLine(100);
                if(farLeftVal ==1){
                    position = PositionList::BLUE_T;
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
        
        void checkOtherSideFromClockwise() {
            Serial.println("Checking other Side From Clockwise");
            while(!(pillPosition == 0)){
                utilityFunction();
                binaryFollowLine(100);
            }
            crossTFromClockwise();
            while(!(distanceFrontVal > 500)){
                utilityFunction();
                binaryFollowLine(100);
            }
            while(!(currentBoxCol != BoxCol::NO_BOX)){
                utilityFunction();
                checkBoxColour();
            }
        }

        void dealWithTwoClockwiseReds(){
            Serial.println("Dealing with Two Reds");
            while(!(position == PositionList::PILL)){
                utilityFunction();
                turnLeft();
                position = PositionList::PILL;
            }
            while(!(distanceFrontVal > 500)){
                utilityFunction();
                binaryFollowLine(100);
            }
            while(!(currentBoxCol != BoxCol::NO_BOX)){
                utilityFunction();
                checkBoxColour();
            }
            while(!(hasBoxAtm==true)){
                utilityFunction();
                pickupBox();
            }
            while(!(clockwise==false)){
                utilityFunction();
                turn180();
                clockwise = false;
            }
            while(!(pillPosition == 0)){
                utilityFunction();
                binaryFollowLine(100);
            }
            crossTFromClockwise();
            while(!(pillPosition == 0)){
                utilityFunction();
                binaryFollowLine(100);
            }
            while(!(onTargetBox==true)){
                utilityFunction();
                binaryFollowLine(100);
            }
            while(!(clockwise==true)){
                utilityFunction();
                placeBox();
                clockwise = true;
            }
            while(!(pillPosition == 0)){
                utilityFunction();
                binaryFollowLine(100);
            }
            crossTFromAnticlock();
            while(!(distanceFrontVal > 500)){
                utilityFunction();
                binaryFollowLine(100);
            }
            while(!(currentBoxCol != BoxCol::NO_BOX)){
                utilityFunction();
                checkBoxColour();
            }
            while(!(hasBoxAtm==true)){
                utilityFunction();
                pickupBox();
            }
            while(!(pillPosition == 0)){
                utilityFunction();
                binaryFollowLine(100);
            }
            while(!(onTargetBox==true)){
                utilityFunction();
                binaryFollowLine(100);
            }
            while(!(clockwise==false)){
                utilityFunction();
                placeBox();
                clockwise = false;
            }
            while(!(pillPosition == 0)){
                utilityFunction();
                binaryFollowLine(100);
            }
            while(!(farRightVal == true)){
                utilityFunction();
                binaryFollowLine(100);
            }
            while(!(position== PositionList::TUNNEL)){
                utilityFunction();
                turnRight();
                position = PositionList::TUNNEL;
            }
            while(!(position == PositionList::FIRST_JUNCTION)){
                utilityFunction();
                binaryFollowLine(100);
                if(farRightVal ==1 ){
                    position = PositionList::FIRST_JUNCTION;
                }
            }
            while(!(position == PositionList::START)){
                utilityFunction();
                binaryFollowLine(100);
                if(farRightVal ==1 && farLeftVal ==1){
                    position = PositionList::START;
                    follow(1000);
                    runMotors(0,0);
                }
            }
            while(true){
                continue;
            }
        }
        
        //picks up blue block turns and turns right at T junction (used for placeblue)
        void ClockwisepickUpAndReturnT(){
            Serial.println("Returning to T");
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
                    position = PositionList::MAIN_T_JUNCTION;
                }
            }
            Serial.println("Main T");
            while(!(position == PositionList::TUNNEL)){
                utilityFunction();
                turnRight();
                direction = Directions::AWAY_FROM_PILL;
                position = PositionList::TUNNEL;
            }
            Serial.println("Tunnel");
            follow(2000);
        }

        //picks up blue block turns and turns left at T junction (used for placeblue)
        void AntiClockpickUpAndReturnT(){
            Serial.println("Returning to T");
            while(!(hasBoxAtm==true)){
                utilityFunction();
                pickupBox();
            }
            while(!(clockwise==true)){
                utilityFunction();
                turn180();
                clockwise = true;
            }
            while(!(pillPosition== 0)){
                utilityFunction();
                binaryFollowLine(100);
            }
            while(!(position == PositionList::MAIN_T_JUNCTION)){
                utilityFunction();
                binaryFollowLine(100);
                if (farLeftVal==1){
                    position = PositionList::MAIN_T_JUNCTION;
                }
            }
            while(!(position == PositionList::TUNNEL)){
                utilityFunction();
                turnLeft();
                direction = Directions::AWAY_FROM_PILL;
                position = PositionList::TUNNEL;
            }
            follow(2000);
        }
        
        //picks up red and places it on clockwise side of T junction before turning round and passing T junction
        void placeRedTemporary(){
            Serial.println("Temporarily placing Red Block");
            while(!(hasBoxAtm==true)){
                utilityFunction();
                pickupBox();
            }
            while(!(clockwise==true)){
                utilityFunction();
                turn180();
                clockwise = true;
            }
            while(!(pillPosition== 0)){
                utilityFunction();
                binaryFollowLine(100);
            }
            crossTFromAnticlock();
            follow(400);
            runMotors(0,0);
            for (pos = servoClose; pos >= servoStart; pos -= 1) { // goes from 0 degrees to 180 degrees
                // in steps of 1 degree
                Servo1.write(pos);              // tell servo to go to position in variable 'pos'
                delay(50);                       // waits 15ms for the servo to reach the position
            }   
            reverseAndTwist();
            clockwise = false;
            
            crossTFromClockwise();

            /*from t junction
            goes forward a bit
            places block
            reverses some and twists
            set clockwise to false
            goes to t juction and past it
            */
        }

        //crosses t junction from clockwise dealing with pill position reset
        void crossTFromClockwise(){
            Serial.println("Crossing T Junction");
            while(!(position == PositionList::MAIN_T_JUNCTION )){
                utilityFunction();
                binaryFollowLine(100);
                if(farRightVal == 1){
                    position = PositionList::MAIN_T_JUNCTION;
                }
            }
            while(!(position == PositionList::PILL)){
                utilityFunction();
                binaryFollowLine(100);
                if( farRightVal == 0){
                    position = PositionList::PILL;
                    pillPosition = 0;
                    clockwise = true;
                }
            }
            clockwise = false;
        }

        //crosses t junction from anticlockwise dealing with pill position reset
        void crossTFromAnticlock(){
            Serial.println("Crossing T Junction");
            while(!(position == PositionList::MAIN_T_JUNCTION )){
                utilityFunction();
                binaryFollowLine(100);
                if(farLeftVal == 1){
                    position = PositionList::MAIN_T_JUNCTION;
                }
            }
            while(!(position == PositionList::PILL)){
                utilityFunction();
                binaryFollowLine(100);
                if( farLeftVal == 0){
                    position = PositionList::PILL;
                    pillPosition = 0;
                    clockwise = false;
                }
            }
            clockwise = true;
        }

        //CLOCK 1 IS RED
        //ANTICLOCK 1 IS RED 
        //ANTICLOKC 2 IS BLUE
        //STARTS FROM TUNNEL
        void testProgram(){
            Serial.println("Running Test Program");
            while(!(position== PositionList::MAIN_T_JUNCTION)){
                utilityFunction();
                binaryFollowLine(100);
                if(farRightVal ==1 && farLeftVal ==1){
                    position = PositionList::MAIN_T_JUNCTION;
                }
            }
            while(!(position == PositionList::PILL)){
                utilityFunction();
                turnRight();
                position = PositionList::PILL;
                clockwise = false;
            }
            while(!(distanceFrontVal > 500)){
                utilityFunction();
                binaryFollowLine(100);
            }
            while(currentBoxCol == BoxCol::NO_BOX){
                utilityFunction();
                checkBoxColour();
                currentBoxCol = BoxCol::RED;
            }
            //JUST CHECKING NOT BLUE BOX
            while(currentBoxCol == BoxCol::BLUE){
                Serial.println("seeing blue in Anti1, must be errror");
            }
            if(currentBoxCol == BoxCol::RED){
                //ANTICLOCK 1 RED
                //TEMPORARY PLACE THEN CHECK ANTI AGAIN
                placeRedTemporary();
                while(!(distanceFrontVal > 500)){
                    utilityFunction();
                    binaryFollowLine(100);
                }
                while(!(currentBoxCol != BoxCol::NO_BOX)){
                    utilityFunction();
                    checkBoxColour();
                    currentBoxCol = BoxCol::BLUE;
                }
                //JUST CHECKING NOT red BOX
                while(currentBoxCol == BoxCol::RED){
                    Serial.println("seeing blue in Anti1, must be errror");
                }
                
                AntiClockpickUpAndReturnT();
                placeFirstBlueBox();
                dealWithTwoClockwiseReds();
            }
        }


        void runProgram(){
            // testProgram();
            //FIRST CHECKS FIRST ANTICLOCK BLOCK
            pickupBox();
            placeFirstBlueBox();
            //placeSecondBlueBox();  
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
            Serial.println("Main T");
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
            while(!(currentBoxCol != BoxCol::NO_BOX)){
                utilityFunction();
                checkBoxColour();
            }
            if(currentBoxCol == BoxCol::BLUE){
                //CLOCKWISE 1 IS BLUE
                //PLACES BLUE AND LOOKS CLOCKWISE AGAIN
                ClockwisepickUpAndReturnT();
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
                while(!(currentBoxCol != BoxCol::NO_BOX)){
                    utilityFunction();
                    checkBoxColour();
                }
                if(currentBoxCol == BoxCol::BLUE){
                    //CLOCKWISE 1 WAS BLUE
                    //CLOCKWISE 2 IS BLUE
                    //ANTICLOCK 1 & 2 RED
                    //PLACES SECOND BLUE THEN DEALS WITH REMAINING REDS
                    ClockwisepickUpAndReturnT();
                    placeSecondBlueBox();
                    while(!(position == PositionList::PILL)){
                        utilityFunction();
                        turnLeft();
                        position = PositionList::PILL;
                    }
                    while(!(distanceFrontVal > 500)){
                        utilityFunction();
                        binaryFollowLine(100);
                    }
                    while(!(currentBoxCol != BoxCol::NO_BOX)){
                        utilityFunction();
                        checkBoxColour();
                    }
                    while(!(hasBoxAtm==true)){
                        utilityFunction();
                        pickupBox();
                    }
                    while(!(pillPosition==1)){
                        utilityFunction();
                        binaryFollowLine(100);
                    }
                    while(!(onTargetBox==true)){
                        utilityFunction();
                        binaryFollowLine(100);
                    }
                    while(!(clockwise==false)){
                        utilityFunction();
                        placeBox();
                        clockwise = false;
                    }
                    checkOtherSideFromClockwise();
                    while(!(hasBoxAtm==true)){
                        utilityFunction();
                        pickupBox();
                    }
                    while(!(clockwise == true)){
                        utilityFunction();
                        turn180();
                        clockwise = true;
                    }
                    while(!(pillPosition == 0)){
                        utilityFunction();
                        binaryFollowLine(100);
                    }
                    crossTFromAnticlock();
                    while(!(onTargetBox==true)){
                        utilityFunction();
                        binaryFollowLine(100);
                    }
                    while(!(clockwise==false)){
                        utilityFunction();
                        placeBox();
                        clockwise = false;
                    }
                    while(!(pillPosition == 0)){
                        utilityFunction();
                        binaryFollowLine(100);
                    }
                    while(!(position == PositionList::FIRST_JUNCTION)){
                        utilityFunction();
                        binaryFollowLine(100);
                        if(farRightVal ==1 ){
                            position = PositionList::FIRST_JUNCTION;
                        }
                    }
                    while(!(position == PositionList::START)){
                        utilityFunction();
                        binaryFollowLine(100);
                        if(farRightVal ==1 && farLeftVal ==1){
                            position = PositionList::START;
                            follow(1000);
                            runMotors(0,0);
                        }
                    }
                    while(true){
                        continue;
                    }
                } else{
                    //CLOCKWISE 1 WAS BLUE
                    //CLOCKWISE 2 IS RED
                    //NOW CHECKING OTHER SIDE
                    while(!(clockwise == false)){
                        utilityFunction();
                        reverseAndTwist();
                        clockwise == false;
                        currentBoxCol = BoxCol::NO_BOX;
                    }
                    checkOtherSideFromClockwise();
                    if(currentBoxCol == BoxCol::BLUE){
                        //CLOCKWISE 1 WAS BLUE
                        //CLOCKWISE 2 IS RED
                        //ANTICLOCK 1 IS BLUE
                        //ANTICLOCK 2 IS RED
                        AntiClockpickUpAndReturnT();
                        placeSecondBlueBox();
                        while(!(position == PositionList::PILL)){
                            utilityFunction();
                            turnRight();
                            position = PositionList::PILL;
                            clockwise = false;
                        }
                        while(!(distanceFrontVal > 500)){
                            utilityFunction();
                            binaryFollowLine(100);
                        }
                        while(!(currentBoxCol != BoxCol::NO_BOX)){
                            utilityFunction();
                            checkBoxColour();
                        }
                        while(!(hasBoxAtm==true)){
                            utilityFunction();
                            pickupBox();
                        }
                        while(!(pillPosition==-1)){
                            utilityFunction();
                            binaryFollowLine(100);
                        }
                        while(!(onTargetBox==true)){
                            utilityFunction();
                            binaryFollowLine(100);
                        }
                        while(!(clockwise==true)){
                            utilityFunction();
                            placeBox();
                            clockwise = true;
                        }
                        while(!(pillPosition==0)){
                            utilityFunction();
                            binaryFollowLine(100);
                        }
                        crossTFromAnticlock();
                        while(!(distanceFrontVal > 500)){
                            utilityFunction();
                            binaryFollowLine(100);
                        }
                        while(!(currentBoxCol != BoxCol::NO_BOX)){
                            utilityFunction();
                            checkBoxColour();
                        }
                        while(!(hasBoxAtm==true)){
                            utilityFunction();
                            pickupBox();
                        }
                        while(!(clockwise ==false)){
                            utilityFunction();
                            turn180();
                            clockwise = false;
                        }
                        while(!(pillPosition==0)){
                            utilityFunction();
                            binaryFollowLine(100);
                        }
                        crossTFromClockwise();
                        while(!(onTargetBox==true)){
                            utilityFunction();
                            binaryFollowLine(100);
                        }
                        while(!(clockwise==true)){
                            utilityFunction();
                            placeBox();
                            clockwise = true;
                        }
                        while(!(pillPosition == 0 )){
                            utilityFunction();
                            binaryFollowLine(100);
                        }
                        while(!(farLeftVal == true)){
                            utilityFunction();
                            binaryFollowLine(100);
                        }
                        while(!(position== PositionList::TUNNEL)){
                            utilityFunction();
                            turnLeft();
                            position = PositionList::TUNNEL;
                        }
                        while(!(position == PositionList::FIRST_JUNCTION)){
                            utilityFunction();
                            binaryFollowLine(100);
                            if(farRightVal ==1 ){
                                position = PositionList::FIRST_JUNCTION;
                            }
                        }
                        while(!(position == PositionList::START)){
                            utilityFunction();
                            binaryFollowLine(100);
                            if(farRightVal ==1 && farLeftVal ==1){
                                position = PositionList::START;
                                follow(1000);
                                runMotors(0,0);
                            }
                        }
                        while(true){
                            continue;
                        }
                    } else{
                        //CLOCKWISE 1 WAS BLUE
                        //CLOCKWISE 2 IS RED
                        //ANTICLOCK 1 IS RED
                        //ANTICLOCK 2 IS BLUE
                        placeRedTemporary();
                        while(!(distanceFrontVal > 500)){
                            utilityFunction();
                            binaryFollowLine(100);
                        }
                        while(!(currentBoxCol != BoxCol::NO_BOX)){
                            utilityFunction();
                            checkBoxColour();
                        }
                        AntiClockpickUpAndReturnT();
                        placeSecondBlueBox();
                        dealWithTwoClockwiseReds();
                    }
                }
            } 
            else {
                //CLOCKWISE 1 IS RED
                //CHECKING OTHER SIDE
                while(!(clockwise == false)){
                    utilityFunction();
                    reverseAndTwist();
                    clockwise = false;
                    currentBoxCol = BoxCol::NO_BOX;
                }
                checkOtherSideFromClockwise();
                if(currentBoxCol == BoxCol::BLUE){
                    //CLOCKWISE 1 IS RED
                    //ANTICLOCK 1 IS BLUE
                    AntiClockpickUpAndReturnT();
                    placeFirstBlueBox();
                    while(!(position == PositionList::PILL)){
                        utilityFunction();
                        turnRight();
                        position = PositionList::PILL;
                    }
                    while(!(distanceFrontVal > 500)){
                        utilityFunction();
                        binaryFollowLine(100);
                    }
                    while(!(currentBoxCol != BoxCol::NO_BOX)){
                        utilityFunction();
                        checkBoxColour();
                    }
                    if(currentBoxCol == BoxCol::BLUE){
                        //CLOCKWISE 1 IS RED
                        //ANTICLOCK 1 WAS BLUE
                        //ANTICLOCK 2 IS BLUE
                        //CLOCKWISE 2 IS RED
                        ClockwisepickUpAndReturnT();
                        placeSecondBlueBox();
                        dealWithTwoClockwiseReds();
                    } else{
                        //CLOCKWISE 1 IS RED
                        //ANTICLOCK 1 WAS BLUE
                        //ANTICLOCK 2 IS RED
                        //SO CLOCKWISE 2 MUST BE BLUE
                        //TAKING RED ACROSS TO T
                        placeRedTemporary();
                        while(!(distanceFrontVal > 500)){
                            utilityFunction();
                            binaryFollowLine(100);
                        }
                        while(!(currentBoxCol != BoxCol::NO_BOX)){
                            utilityFunction();
                            checkBoxColour();
                        }
                        AntiClockpickUpAndReturnT();
                        placeSecondBlueBox();
                        dealWithTwoClockwiseReds();
                    }
                }
                else{
                    //CLOCKWISE 1 IS RED
                    //ANTICLOCK 1 IS RED
                    //CLOCKWISE 2 IS BLUE
                    //ANTICLOCK 2 IS BLUE
                    placeRedTemporary();
                    while(!(distanceFrontVal > 500)){
                        utilityFunction();
                        binaryFollowLine(100);
                    }
                    while(!(currentBoxCol != BoxCol::NO_BOX)){
                        utilityFunction();
                        checkBoxColour();
                    }
                    AntiClockpickUpAndReturnT();
                    placeFirstBlueBox();
                    while(!(position == PositionList::PILL)){
                        utilityFunction();
                        turnRight();
                        position = PositionList::PILL;
                    }
                    while(!(distanceFrontVal > 500)){
                        utilityFunction();
                        binaryFollowLine(100);
                    }
                    while(!(currentBoxCol != BoxCol::NO_BOX)){
                        utilityFunction();
                        checkBoxColour();
                    }
                    ClockwisepickUpAndReturnT();
                    placeSecondBlueBox();
                    dealWithTwoClockwiseReds();
                }
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

    Servo1.write(servoStart);
    delay(10);
}




void loop() {
    Bot.runProgram();
}

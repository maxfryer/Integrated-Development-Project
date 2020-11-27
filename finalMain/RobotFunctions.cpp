#ifndef SENSORS
#include  "SensorClass.cpp"
#endif
#define SENSORS



class Robot: public Sensors {
    public:
        
        /* MOTORS */
        float motorSpeed = 100; //EDIT THIS 
        float lineFollowDampingFactor = 0.9;
        float motorSpeedLeft;
        float motorSpeedRight;
        float speedDifference = 0;
        int pos = 85;  // variable to store the servo position


        /*DEALING WITH BOXES*/
        bool boxBeingColourChecked = false;
        bool hasBoxAtm = false;
        int pillPosition = 0;
        bool onTargetBox = false;


        bool run = false;

        enum class ActionType { LINE, TURN_LEFT, TURN_RIGHT, TURN_180, ADVANCE_TO_HOME, CHECK_BOX};
        enum class PositionList { START_BOX,START,FIRST_JUNCTION,TUNNEL,MAIN_T_JUNCTION,PILL,BLUE_TRACK,BLUE_SQUARE };
        enum class Directions {TOWARDS_PILL,AWAY_FROM_PILL};
        enum class BoxCol {RED,BLUE,NO_BOX};

        ActionType currentRoutine = ActionType::LINE;
        PositionList position = PositionList::TUNNEL;//START_BOX;
        Directions direction = Directions::AWAY_FROM_PILL;//TOWARDS_PILL;
        BoxCol currentBoxCol = BoxCol::BLUE;



        void turnLeft() {

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
            return;
        }

        void binaryFollowLine(int increaseRate) { 

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
};
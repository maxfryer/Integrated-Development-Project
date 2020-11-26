#include "SensorClass.ino"

class Robot: public Sensors {
    public:
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


    void runMotors(){
        
    }
}
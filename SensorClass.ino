class Sensors{
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

        /* THRESHOLDS */
        int lineSensorThreshold = 300;
        float distanceSensorThreshold = 460;

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
}
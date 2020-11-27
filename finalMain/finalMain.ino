#ifndef STRUCTURE
#include  "finalStructure.cpp"
#endif
#define STRUCTURE

TaskManager Bot;

void setup(){
    Bot.AFMS.begin();
    Serial.begin(9600);
    Bot.Servo1.attach(10);

    pinMode(Bot.startButtonPin, INPUT);
    pinMode(Bot.offAxisLeft,INPUT);
    pinMode(Bot.offAxisRight,INPUT);

    pinMode(Bot.ledPinFirst,OUTPUT);
    pinMode(Bot.ledPinSecond,OUTPUT);
    pinMode(Bot.ledPinThird,OUTPUT);

}

void loop(){
    if(Bot.run == true){
        Bot.checkAllSensorValues(false);
        Bot.flashLEDS();
        Bot.runTasks();
    }
}
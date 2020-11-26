// Include the Servo library 
#include <Servo.h> 
// Declare the Servo pin, 9 for servo2 and 10 for servo1
int servoPin = 10; 

int pos =0;  // variable to store the servo position

// Create a servo object 
Servo Servo1;

void setup() {
  // put your setup code here, to run once:
  Servo1.attach(servoPin);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    Servo1.write(pos);              // tell servo to go to position in variable 'pos'
    delay(200);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    Servo1.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

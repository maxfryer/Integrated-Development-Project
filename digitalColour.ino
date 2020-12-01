int sensorPin =4;    // select the input pin for the potentiometer

int sensorValue = 0;  // variable to store the value coming from the sensor

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);           //Start serial and set the correct Baud Rate
}

void loop() {
  // put your main code here, to run repeatedly:
  sensorValue = digitalRead(sensorPin);
  Serial.println(sensorValue);

}

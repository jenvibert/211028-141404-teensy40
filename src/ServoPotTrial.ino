# include <Arduino.h> 
# include <Servo.h>

Servo myservo;  //initialize servo name
int Pot_Input = 21;  //pot input at pin 21 on teensy

void setup() {
  pinMode(Pot_Input, INPUT);  //set pot as an input
  myservo.attach(13);  //servo signal pin connected to pin 13
  Serial.begin (9600);  //arduino setup

} 

void loop() {
  int val = analogRead(Pot_Input);  //read value from pot
  int angle = map(val,1,1023,0,180);  //converting analog potentiometer range to angle range
  Serial.println(angle);  //print angle to serial monitor
  myservo.write(angle);  //send angle to servo
}

# include <Arduino.h>
# include <Servo.h>

Servo myservo;
int Pot_Input = 21;    

void setup() {
  pinMode(Pot_Input, INPUT);  
  myservo.attach(13);
  Serial.begin (9600);

}

void loop() {
  int val = analogRead(Pot_Input); 
  int angle = map(val,1,1023,0,270);
  Serial.println(angle);
  myservo.write(angle);
}

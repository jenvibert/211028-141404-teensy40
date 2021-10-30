# include <Arduino.h>

int Pot_Input = 21;   
int Motor_Output = 13; 

void setup() {
  pinMode(Pot_Input, INPUT);  
  pinMode(Motor_Output, OUTPUT);
}

void loop() {
  int val = analogRead(Pot_Input); 
  int angle = map(val,1,1023,0,270);
  Serial.println(angle);
}

// hello
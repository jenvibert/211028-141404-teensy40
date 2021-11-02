# include <Arduino.h>

int LEDPin = 6;
int push_button = 10;

void setup() {
    pinMode(LEDPin, OUTPUT);
    pinMode(push_button, INPUT);
}

void loop(){
    int Button_State = digitalRead(push_button);
    if (Button_State == HIGH){
        digitalWrite(LEDPin, HIGH);
    }
    else{
        digitalWrite(LEDPin, LOW);
    }
}
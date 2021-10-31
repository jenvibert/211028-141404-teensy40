#include <Arduino.h>
#include <Servo.h>

//Threshold for servo motor control with muscle sensor.
//You can set a threshold according to the maximum and minimum values of the muscle
#define THRESHOLD 200

//pin number where the sensor is connected. (Analog 0)
#define EMG_PIN 14

//pin number where the servo is connected
#define SERVO_PIN 13

//Define Servo motor
Servo SERVO_1;

//-------------------- VOID SETUP  ------------------

void setup(){
    //BAUDRATE set to 9600
    Serial.begin(9600);

    //set servo motor to digital pin 13
    SERVO_1.attach(SERVO_PIN);
}

//-------------------- VOID LOOP -------------------
void loop(){
    //The "value" variable reads the value from the analog pin to which sensor
    int value = analogRead(EMG_PIN);

    //if the sensor value is greater than threshold, the servo will turn
    if (value > THRESHOLD){
        SERVO_1.write(179);
    }

    //if the sensor is less than the threshold servo will turn to 0 degrees
    else{
        SERVO_1.write(0);
    }

    //you can use serial port monitor to set threshold properly
    Serial.println(value);
}
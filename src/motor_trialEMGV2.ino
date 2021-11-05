
#include "SparkFun_TB6612.h"
#include <Arduino.h>


#define DIR1 17 // YELLOW DIR1 ON MOTOR DRIVER
#define PWM1 22
#define EMG 23

const int OffsetA = 1;

void setup()
{
  pinMode(EMG,INPUT);
  Serial.begin(9600);
}
 
 
void loop()
{
  Motor motor1 = Motor(DIR1,PWM1,OffsetA);
   //Use of the drive function which takes as arguements the speed
   //and optional duration.  A negative speed will cause it to go
   //backwards.  Speed can be from -255 to 255.  Also use of the 
   //brake function which takes no arguements.
   motor1.drive(255,1000);
   motor1.drive(-255,1000);
   motor1.brake();
   delay(1000);
   
   int val = analogRead(EMG);
   int angle = map(val,1,1023,0,180);
   Serial.println(angle);
   //Reading EMG value

   //Use of the forward function, which takes as arguements two motors
   //and optionally a speed.  If a negative number is used for speed
   //it will go backwards
   forward(motor1, 150);
   delay(1000);
   
   //Use of the back function, which takes as arguments two motors 
   //and optionally a speed.  Either a positive number or a negative
   //number for speed will cause it to go backwards
   back(motor1, -150);
   delay(1000);
   
   //Use of the brake function which takes as arguments two motors.
   //Note that functions do not stop motors on their own.
   brake(motor1);
   delay(1000);
   
   //Use of the left and right functions which take as arguements two
   //motors and a speed.  This function turns both motors to move in 
   //the appropriate direction.  For turning a single motor use drive.
   drive(motor1, 100);
   delay(1000);
   
   //Use of brake again.
   brake(motor1);
   delay(1000);
   
}
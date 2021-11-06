
#include "SparkFun_TB6612.h"
#include <Arduino.h>

#define DIR1 17 // YELLOW DIR1 ON MOTOR DRIVER
#define PWM1 13
#define EMG 23
#define DIR2 16

int PrevPos = 0;
int DesPos = 0;

const int OffsetA = 1;
Motor motor1 = Motor(DIR1,DIR2,PWM1,OffsetA);
Motor motor2 = Motor(DIR1,DIR2,PWM1,OffsetA);

void setup()
{
  pinMode(EMG,INPUT);
  Serial.begin(9600);
}
 
 
void loop()
{
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

 while (PrevPos = 0){
   forward(motor1,motor2, 10);
   delay(1000);
   PrevPos++;
 }

 while (DesPos > PrevPos){
   forward(motor1,motor2, 10);
   delay(1000);
   PrevPos++;
 }

 while (DesPos < PrevPos){
   back(motor1,motor2, -10);
   delay(1000);
   DesPos++;
 }

 while (DesPos = PrevPos){
   brake(motor1, motor2);
   delay(1000);
 break;
 }

   PrevPos = DesPos;
   DesPos = angle;

}
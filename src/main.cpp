
#include "SparkFun_TB6612.h"
#include <Arduino.h>

/****************INFORMATION***************************
// Since we are running the motor at 5V, the MAX RPM we can get is 83.33
// Speed can stay constant and the input variable adjusts the delay amount
// We found that manipulating trhe delay would help manipulate the angle
// PWM = 127.5 EXPECTED: 250 Degrees -- for a 2 second delay we were getting 150 degrees 
    // -- for 5 second delay we are getting around 360 degrees
*******************************************************/

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

/*******READING EMG***********/
   int val = analogRead(EMG); 
   int angle = map(val,1,1023,0,180);

/*******DESPOS ADJUSTING******/
  if (angle <= 180 && angle > 135){
     DesPos = 180;
     }
  else if(angle <=135 && angle > 90){
     DesPos = 135;
     }
  else if(angle <= 90 && angle > 45){
     DesPos = 90;
     }
  else if(angle <= 45 && angle > 2){
     DesPos = 45;
     }
  else{
    DesPos = 0;
  }

Serial.println("DESPOS IS");
Serial.println(DesPos);
delay(1000);
Serial.println("PREVPOS IS");
Serial.println(PrevPos);

/*************FOR DESPOS = 0*********/
  if (DesPos == 0) {
     if (PrevPos == 0){
         motor1.brake();
       }
     else if (PrevPos == 45){
         motor1.drive(-127.5);
         delay(625);
       }
     else if (PrevPos == 90){
         motor1.drive(-127.5);
         delay(1250);
       }
     else if (PrevPos == 135){
         motor1.drive(-127.5);
         delay(1875);
       }
     else{
         motor1.drive(-127.5);
         delay(2500);
       }
       PrevPos = DesPos;
     }

/*************FOR DESPOS = 45*********/
 if (DesPos == 45) {
    if (PrevPos == 45){
        motor1.brake();
      }
    else if (PrevPos == 90){
        motor1.drive(-127.5);
        delay(625);
      }
    else if (PrevPos == 180){
        motor1.drive(-127.5);
        delay(1875);
      }
    else if (PrevPos == 135){
        motor1.drive(-127.5);
        delay(1250);
      }
    else{
        motor1.drive(127.5);
        delay(625);
      }
       PrevPos = DesPos;
    }

/*************FOR DESPOS = 90*********/
 if (DesPos == 90) {
    if (PrevPos == 90){
        motor1.brake();
      }
    else if (PrevPos == 45){
        motor1.drive(127.5);
        delay(625);
      }
    else if (PrevPos == 180){
        motor1.drive(-127.5);
        delay(1250);
      }
    else if (PrevPos == 135){
        motor1.drive(-127.5);
        delay(625);
      }
    else{
        motor1.drive(127.5);
        delay(1250);
      }
       PrevPos = DesPos;
    }

/*************FOR DESPOS = 135*********/
 if (DesPos == 135) {
    if (PrevPos == 135){
        motor1.brake();
      }
    else if (PrevPos == 45){
        motor1.drive(127.5);
        delay(1250);
      }
    else if (PrevPos == 180){
        motor1.drive(-127.5);
        delay(625);
      }
    else if (PrevPos == 90){
        motor1.drive(127.5);
        delay(625);
      }
    else{
        motor1.drive(127.5);
        delay(1875);
      }
       PrevPos = DesPos;
    }

/*************FOR DESPOS = 180*********/
    if (DesPos == 180) {
    if (PrevPos == 180){
        motor1.brake();
      }
    else if (PrevPos == 45){
        motor1.drive(127.5);
        delay(1875);
      }
    else if (PrevPos == 135){
        motor1.drive(127.5);
        delay(625);
      }
    else if (PrevPos == 90){
        motor1.drive(127.5);
        delay(1250);
      }
    else{
        motor1.drive(127.5);
        delay(2500);
      }
       PrevPos = DesPos;
    }
//END OF VOID LOOP
}


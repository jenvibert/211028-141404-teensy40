
#include "SparkFun_TB6612.h"
#include <Arduino.h>

//****** INFORMATION
// Since we are running the motor at 5V, the MAX RPM we can get is 83.33
// Speed can stay constant and the input variable adjusts the delay amount
// We found that manipulating trhe delay would help manipulate the angle
//******

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
   //backwards.  Speed can be from -255 to 255 (PWM).  Also use of the 
   //brake function which takes no arguements.
    motor1.drive(127.5);
    delay(2500);
    motor1.drive(-127.5);
    delay(2500);
    motor1.brake();
    delay(1000);
    
    // PWM = 127.5 EXPECTED: 250 Degrees -- for a 2 second delay we were getting 150 degrees 
    // -- for 5 second delay we are getting around 360 degrees

   int val = analogRead(EMG); 
   int angle = map(val,1,1023,0,270);
   Serial.println(angle);
   //Reading EMG value

    PrevPos = DesPos;
    DesPos = angle;

   
   while (PrevPos = 0){
     Serial.println("In first Loop");
     forward(motor1,motor2, 30);
     delay(1000);
     PrevPos++;
     val = analogRead(EMG);
     angle = map(val,1,1023,0,270);
     Serial.println(angle);
   }

   while (DesPos > PrevPos){
     Serial.println("In Second Loop");
     motor1.drive(40,1000);
     forward(motor1,motor2,40);
     delay(1000);
     PrevPos++;
     val = analogRead(EMG);
     angle = map(val,1,1023,0,270);
     Serial.println(angle);
   }

   while (DesPos < PrevPos){
     Serial.println("In Third Loop");
     motor1.drive(-30, 1000);
     //back(motor1,motor2, -20);
     delay(1000);
     DesPos++;
     val = analogRead(EMG);
     angle = map(val,1,1023,0,270);
     Serial.println(angle);
   }

   while (DesPos = PrevPos){
     Serial.println("In Break Loop");
     brake(motor1, motor2);
     delay(1000);
     val = analogRead(EMG);
     angle = map(val,1,1023,0,270);
     Serial.println(angle);
    break;
   }
}


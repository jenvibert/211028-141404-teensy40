#include <Arduino.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define ENCA 21 // YELLOW
#define ENCB 20 // WHITE
#define PWM 13
#define IN2 16
#define IN1 17
#define EMG 23

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/ 

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(EMG,INPUT);
  
  Serial.println("target pos");
}

void loop() {

  // set target position
  //int target = 1200;
  //int target = 250*sin(prevT/1e6);
  int val = analogRead(EMG);
  int target = map(val,1,1023,0,180);

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }
  
  // error
  int e = pos - target;


  // motor power
  float pwr = 255;

  // motor direction
  int dir = 1;
  if(e<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);


  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
  //Serial.println("POSI VALUE");
  //Serial.println(posi);
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
} 

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}
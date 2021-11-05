#ifndef SPARKFUN_TB6612_h
#define SPARKFUN_TB6612_h
#include <Arduino.h>

 class Motor
{
  public:
    Motor(int In1pin, int PWMpin, const int offset);      
    //void drive(Motor motor1,int speed);  
    void drive(int speed, int duration);  

	//Stops motor by setting both input pins high
    void brake(); 
	
  private:
	//
	int In1, PWM, Offset;
	//private functions that spin the motor CC and CCW
	void fwd(int speed);
	void rev(int speed);
  
};
void drive(Motor, int speed);

void forward(Motor, int speed);
//void forward(Motor motor1);

void back(Motor, int speed);
//void back(Motor motor1);

void brake(Motor);

#endif
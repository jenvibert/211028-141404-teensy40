#include <Arduino.h>
#include "FDC1004.h"
#include <Wire.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro


#define ENCA 21 // YELLOW
#define ENCB 20 // WHITE
#define PWM 13
#define IN2 16
#define IN1 17
#define FMG 23

//int led = 13; // assign led pin
// device array to hold all connected i2c devices and analog outputs. board is only capable of 24 I2C devices and 2 analog outputs so we initialize array of 26
// device array indices: [j][0] = I2C address [j][1] = bus [j][2] = capdac 
int deviceArray[24][3] = {0}; 
int led = 14; // assign led pin

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;

// options for FDC1004 setup
#define UPPER_BOUND 0X4000 // max readout capacitance
#define LOWER_BOUND (-1 * UPPER_BOUND)
#define CHANNEL 0    // channel to be read
#define MEASURMENT 0 // measurment channel
char result[100];
char userInput;
FDC1004 FDC;
int sensorCount = 0;

// make a global array of cap values that are just updated for each loop iteration (otherwise need to create arrays each loop iteration

float capValues[24][3] = {0};

// get time

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

// functions to configure and read FDC data
void configureMeasurementonFDCwithAddressAndBus(TwoWire &bus, int addr, int capdac, int i) // the i here is the sensor index in the array - this is where cap dac values are stored.
{
  FDC.configureMeasurementSingle(MEASURMENT, CHANNEL, capdac, bus, addr);
  FDC.triggerSingleMeasurement(MEASURMENT, FDC1004_400HZ, bus, addr);
}

float getReadingFromFDCwithAddressAndBus(TwoWire &bus, int addr, int capdac, int i) // the i here is the sensor index in the array - this is where cap dac values are stored.
{
  uint16_t value[2];
  if (!FDC.readMeasurement(MEASURMENT, value, bus, addr))
  {
    int16_t msb = (int16_t)value[0];
    int32_t capacitance = ((int32_t)457) * ((int32_t)msb); //in attofarads
    capacitance /= 1000;                                   //in femtofarads
    capacitance += ((int32_t)3028) * ((int32_t)capdac);

    if (msb > UPPER_BOUND) // adjust capdac accordingly
    {
      if (capdac < FDC1004_CAPDAC_MAX)
        capdac++;
    }
    else if (msb < LOWER_BOUND)
    {
      if (capdac > 0)
        capdac--;
    }
    
    deviceArray[i][2] = capdac;

    return capacitance;
  }
  else
  {
    return 0;
  }
}

// I2C Scanner to retrieve all device ids

int I2Cscanner()
{
  byte error, address;
  int nDevices = 0;

  for (address = 1; address < 127; address++) // loop through all I2C addresses
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
  

    if (error == 0) // if no I2C errors at the searched address, a device was found.
    {
      // a device was found at this address without any errors. now read it again and see what the devID and manID is to make sure its a sensor.
      Serial.print("current address:");
      Serial.println(address, HEX);
      uint16_t devID = FDC.getDeviceID(Wire, address);

      if (devID == 0x1004)
      {
        int i;
        for (i = 0; i < 24; i++)
        {
          if (deviceArray[i][0] == 0) // if the deviceArray address is zero, add a sensor into the array at that index. this should add sensors in order.
          {
            deviceArray[i][0] = address;
            deviceArray[i][1] = 0;
            break;
          }
        }
        nDevices++;
      }
    }
    else if (error == 4)
    {

      Serial.print(F("Unknown error at address 0x"));
      if (address < 16)
      {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }

  return nDevices;
}

// method to handle input

bool handleInput()
{
  while (Serial.available() > 0)
  {
    char incomingCharacter = Serial.read();
    switch (incomingCharacter)
    {
    case 'y':
      return true;
      break;

    case 'n':
      return false;
      break;

    default:
      break;
      return false;
    }
  }

  return false;
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


void setup() {
  // put your setup code here, to run once:
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(FMG,INPUT);
  
  Serial.println("target pos");
  // initialize the LED pin as an output.
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH); // turn the LED on (HIGH is the voltage level) while set up is ongoing


  // Serial INIT
  Serial.begin(115200); // serial baud rate
  Serial.setTimeout(2);

  // I2C Bus init
  Wire.begin();

  Wire.setClock(400000);

  delay(500);

  // blink LED once to signify set up started
  digitalWrite(led, LOW); 
  delay(250);
  digitalWrite(led, HIGH); 
  delay(250);
  digitalWrite(led, LOW); 
  delay(250);

  // scan for sensors and populate deviceArray
  int numberOfSensors;

  while (true)
  {
    int j;

    for (j = 0; j < 24; j++)
    {
      deviceArray[j][0] = 0;
      deviceArray[j][1] = 0;
    }

    int sensorsBusOne = I2Cscanner();
    numberOfSensors = sensorsBusOne;
    sensorCount = numberOfSensors; // store number of sensors detected in a global variable so you dont have to keep counting number of sensors in main loop (per previous code)
    Serial.println(sensorCount);
    // when at least one sensor is detected, break the loop. If no sensors detected, try to scan again until sensors are detected. (i.e. if unknown error occurs)
    if (numberOfSensors > 0 && numberOfSensors < 25)
      break;
  }


  digitalWrite(led, HIGH); // set Led high to show that setup complete

  // here we should have a device array full of FDC device ID and bus
  // setup complete.
}



void loop() 
{
  

      // data collection part: sample the sensors, and print them in the necessary key val format to the serial port

      int i;

      for (i = 0; i < sensorCount; i++)
      {

        int addr = deviceArray[i][0]; // get the I2C address
        int capdac = deviceArray[i][2]; // get the capdac value


          configureMeasurementonFDCwithAddressAndBus(Wire, addr, capdac, i);

      }

      delay(3); // delay 3 ms to let FDC capture data


      for (i = 0; i < sensorCount; i++)
      {

        int addr = deviceArray[i][0]; // get the I2C address
        int capdac = deviceArray[i][2]; // get the capdac value

        long cap = 0;
        // configure measurement at specified address and bus from device ID
        
        cap = getReadingFromFDCwithAddressAndBus(Wire, addr, capdac, i);


        capValues[i][2] = cap;
        capValues[i][0] = addr;
        capValues[i][1] = 0;


        Serial.println(cap);

    // set target position
    int val= analogWrite(FMG,cap);
    int target = map(val,1,1023,0,180);
  
    // PID constants
    float kp = 1;
    float kd = 0.025;
    float ki = 0.0;

    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Read the position in an atomic block to avoid a potential
    // misread if the interrupt coincides with this code running
    // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
    int unmappedpos = 0; 
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    unmappedpos = posi;
    }
  
    // new postion with mapped encoder
    int pos = map(unmappedpos,0,2527,0,180);
  
    // error
    int e = target - pos;

    // derivative
    float dedt = (e-eprev)/(deltaT);

    // integral
    eintegral = eintegral + e*deltaT;

    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;

    // motor power
    float pwr = 255;

    // motor direction
    //int dir=0;
    //if ((u <= (5)) && (u >= (-5))){
    //  pwr=0;
    //}
    //else if(u<0){
    //  dir = -1;
    //}
    //else if (u>0){
    //  dir = 1;
    //}

    // motor direction
    int dir = 1;
    if(u<0){
    dir = -1;
    }

    // signal the motor
    setMotor(dir,pwr,PWM,IN1,IN2);


    // store previous error
    eprev = e;

    Serial.print(target);
    Serial.print(" ");
    Serial.print(pos);
    Serial.println();
}
        





    
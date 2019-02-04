#include "nimble_control.h"
#include <Wire.h>
#include "motor.h"


void PCF8574Write(byte data);
byte PCF8574Read();

TRSensors trs = TRSensors();
unsigned int sensorValues[NUM_SENSORS];

void nimble_control_init()
{
    Wire.begin();
}

void nimble_control_wait()
{
  byte value = 0x00;
  while(value != 0xEF)
  {
    PCF8574Write(0x1F | PCF8574Read());
    value = PCF8574Read() | 0xE0;
  }
}

void PCF8574Write(byte data)
{
  Wire.beginTransmission(Addr);
  Wire.write(data);
  Wire.endTransmission(); 
}

byte PCF8574Read()
{
  int data = -1;
  Wire.requestFrom(Addr, 1);
  if(Wire.available()) {
    data = Wire.read();
  }
  return data;
}

void nimble_control_calibrate()
{
    for (int i = 0; i < 100; i++)
  {
    if(i<25 || i >= 75)
    {
      nimble_motor_setSpeeds(80,-80);
    }
    else
    {
        nimble_motor_setSpeeds(-80,80);
    }
    trs.calibrate();
  }
  nimble_motor_setSpeeds(0,0); 
}

unsigned int* nimble_control_get_min_ptr()
{
  return trs.calibratedMin;
}

unsigned int* nimble_control_get_max_ptr()
{
  return trs.calibratedMax;
}

unsigned int nimble_control_get_position()
{
  return trs.readLine(sensorValues);
}

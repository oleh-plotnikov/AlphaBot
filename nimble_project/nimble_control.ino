#include "nimble_control.h"
#include <Wire.h>
#include "motor.h"


void PCF8574Write(byte data);
byte PCF8574Read();

TRSensors trs = TRSensors();

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

void nimble_control_calibrate(unsigned int speed)
{
    for (int i = 0; i < 170; i++)
  {
    if(i< 50 || i >= 150)
    {
      nimble_motor_setSpeeds(speed,-speed);
    }
    else
    {
      nimble_motor_setSpeeds(-speed,speed);
    }
    trs.calibrate();
  }
      nimble_motor_setSpeeds(0,0);
      delay(1000);
}

unsigned int* nimble_control_get_min_ptr()
{
  return trs.calibratedMin;
}

unsigned int* nimble_control_get_max_ptr()
{
  return trs.calibratedMax;
}

unsigned int nimble_control_get_position(unsigned int *sensorValues)
{
  return trs.readLine(sensorValues);
}

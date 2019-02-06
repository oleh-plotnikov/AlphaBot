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
  nimble_motor_setSpeeds(25, -25);
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

void follow_segment()
{
  int last_proportional = 0;
  long integral=0;

  while(1)
  {
    // Normally, we will be following a line.  The code below is
    // similar to the 3pi-linefollower-pid example, but the maximum
    // speed is turned down to 60 for reliability.

    // Get the position of the line.
    unsigned int position = trs.readLine(sensorValues);

    // The "proportional" term should be 0 when we are on the line.
    int proportional = ((int)position) - 2000;

    // Compute the derivative (change) and integral (sum) of the
    // position.
    int derivative = proportional - last_proportional;
    integral += proportional;

    // Remember the last position.
    last_proportional = proportional;

    // Compute the difference between the two motor power settings,
    // m1 - m2.  If this is a positive number the robot will turn
    // to the left.  If it is a negative number, the robot will
    // turn to the right, and the magnitude of the number determines
    // the sharpness of the turn.
    int power_difference = proportional/20 + integral/10000 + derivative*10;

    // Compute the actual motor settings.  We never set either motor
    // to a negative value.
    const int maximum = 150; // the maximum speed
    if (power_difference > maximum)
      power_difference = maximum;
    if (power_difference < -maximum)
      power_difference = - maximum;

    if (power_difference < 0)
    {
      analogWrite(PWMA,maximum + power_difference);
      analogWrite(PWMB,maximum);
    }
    else
    {
      analogWrite(PWMA,maximum);
      analogWrite(PWMB,maximum - power_difference);
    }

    // We use the inner three sensors (1, 2, and 3) for
    // determining whether there is a line straight ahead, and the
    // sensors 0 and 4 for detecting lines going to the left and
    // right.
   if(millis() - lasttime >100)
   {
    if (sensorValues[1] < 150 && sensorValues[2] < 150 && sensorValues[3] < 150)
    {
      // There is no line visible ahead, and we didn't see any
      // intersection.  Must be a dead end.
    //  SetSpeeds(0,0);
      return;
    }
    else if (sensorValues[0] > 600 || sensorValues[4] > 600)
    {
      // Found an intersection.
    //  SetSpeeds(0, 0);
      return;
    }
   }
  }
}
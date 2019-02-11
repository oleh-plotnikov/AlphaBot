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
    const int maximum = 110; // the maximum speed 110
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

    if (sensorValues[1] < 150 && sensorValues[2] < 150 && sensorValues[3] < 150)
    {
      // There is no line visible ahead, and we didn't see any
      // intersection.  Must be a dead end.
    //  nimble_motor_setSpeeds(0,0);
      return;
    }
    else if (sensorValues[0] > 600 || sensorValues[4] > 600)
    {
      // Found an intersection.
    //  nimble_motor_setSpeeds(0, 0);
      return;
    }
  }
}

unsigned char select_turn(unsigned char found_left, unsigned char found_straight, unsigned char found_right, unsigned int decision)
{
  if(decision == DECISION_LEFT)
  {
    if (found_left)
      return 'L';
    else if (found_straight)
      return 'S';
    else if (found_right)
      return 'R';
    else
      return 'B';
  }
  else
  {
    if (found_right)
      return 'L';
    else if (found_straight)
      return 'S';
    else if (found_left)
      return 'R';
    else
      return 'B';
  }
}

void turn(unsigned char dir)
{
 // if(millis() - lasttime >500)
  {
    switch(dir)
    {
    case 'L':
      // Turn left.
      nimble_motor_setSpeeds(-100, 100);
      delay(175);
      break;
    case 'R':
      // Turn right.
      nimble_motor_setSpeeds(100, -100);
      delay(175); // Last correct
      break;
    case 'B':
      // Turn around.
      nimble_motor_setSpeeds(75, -75);
      delay(410); // Last correct
      break;
    case 'S':
      // Don't do anything!
      break;
    }
  }
  nimble_motor_setSpeeds(0, 0);
 // value = 0;
//  while(value != 0xEF)  //wait button pressed
//  {
//    PCF8574Write(0x1F | PCF8574Read());
//    value = PCF8574Read() | 0xE0;
//  }
  //Serial.write(dir);
//  Serial.println();

}
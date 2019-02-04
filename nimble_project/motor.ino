#include "motor.h"
  
void nimble_motor_init()
{
  nimble_motor_setSpeeds(0, 0);
}

void nimble_motor_setSpeeds(int Aspeed,int Bspeed)
{
  if(Aspeed < 0)
  {
    digitalWrite(AIN1,HIGH);
    digitalWrite(AIN2,LOW);
    analogWrite(PWMA,-Aspeed);      
  }
  else
  {
    digitalWrite(AIN1,LOW); 
    digitalWrite(AIN2,HIGH);
    analogWrite(PWMA,Aspeed);  
  }
  
  if(Bspeed < 0)
  {
    digitalWrite(BIN1,HIGH);
    digitalWrite(BIN2,LOW);
    analogWrite(PWMB,-Bspeed);      
  }
  else
  {
    digitalWrite(BIN1,LOW); 
    digitalWrite(BIN2,HIGH);
    analogWrite(PWMB,Bspeed);  
  }
}
#ifndef MOTOR_H_
#define MOTOR_H_

#define PWMA   6           //Left Motor Speed pin (ENA)
#define AIN2   A0          //Motor-L forward (IN2).
#define AIN1   A1          //Motor-L backward (IN1)
#define PWMB   5           //Right Motor Speed pin (ENB)
#define BIN1   A2          //Motor-R forward (IN3)
#define BIN2   A3          //Motor-R backward (IN4)

void nimble_motor_init();
void nimble_motor_setSpeeds(int Aspeed,int Bspeed);

#endif
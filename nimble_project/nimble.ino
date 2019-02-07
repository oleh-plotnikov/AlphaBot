/*
 *
 */
#include "nimble.h"
#include "motor.h"
#include "nimble_control.h"
#include "nimble_display.h"

static nimble_state_t nimble_state;
unsigned int sensorValues[NUM_SENSORS];

bool nimble_go_to_state(nimble_state_t state);
void nimble_print_calibrated_val();

void nimble_init()
{
    nimble_display_init();
    nimble_control_init();
    nimble_motor_init();
    nimble_go_to_state(STATE_SETUP);
}

void nimble_run()
{
    int a; 
    char b[5];
    String str;
    static unsigned int calibrate_speed = 50;

    switch(nimble_state)
    {
        case STATE_SETUP:
            nimble_control_wait();
            nimble_go_to_state(STATE_CALIBRATION);
            break;
        case STATE_CALIBRATION:
            nimble_control_calibrate(calibrate_speed);
            nimble_print_calibrated_val();
//            nimble_go_to_state(STATE_GO);

//            nimble_debug_print("Press Button", eNIMBLE_LCD);
            //nimble_control_wait();

            nimble_motor_setSpeeds(35, -35);
            
            for(unsigned int count = 0; count < 100000; count++)
            {
              a = (int)nimble_control_get_position(sensorValues);

              if(a >= 1995 && a <= 2010)
			  {
                nimble_motor_setSpeeds(-50, 50);
                delay(10);
                nimble_motor_setSpeeds(28, -28);
                
                a = (int)nimble_control_get_position(sensorValues);
                if(a >= 1990 && a <= 2010)
                {
                  nimble_motor_setSpeeds(0,0);
                  str = String(a);
                  str.toCharArray(b,5);
                  nimble_debug_print(b, eNIMBLE_LCD);

                  delay(10000);
                  while(1)
                  {
                    a = (int)nimble_control_get_position(sensorValues); 
                    str = String(a);//sensorValues[0]);
                    str.toCharArray(b,5);
                    nimble_debug_print(b, eNIMBLE_LCD);
                  }
                }
              }
            }
            calibrate_speed -= 5;
            break;
        case STATE_WAIT_START:
            break;
        case STATE_GO:
            //nimble_debug_print("STOP", eNIMBLE_LCD);
            nimble_motor_setSpeeds(0,0);
            break;
        case STATE_STOP:
            break;
        case STATE_MAKE_DECISION:
            break;
        case STATE_FINISH:
            break;
        default:
            break;
    }
}

bool nimble_go_to_state(nimble_state_t state)
{
    nimble_state = state;
    return true;
}

void nimble_print_calibrated_val()
{
    unsigned int* calibrated_min;
    unsigned int* calibrated_max;

    calibrated_min = nimble_control_get_min_ptr();
    calibrated_max = nimble_control_get_max_ptr();

    Serial.println("------------------");
    Serial.println("Calibrated");
    uint8_t count = 0;
    Serial.print("MAX: ");
    while(count < NUM_SENSORS)
    {
        Serial.print(" ");
        Serial.print(calibrated_max[count]);
        count++;
    }
    Serial.println("");
  
    count = 0;
    Serial.print("MIN: ");
    while(count < NUM_SENSORS)
    {
      Serial.print(" ");
      Serial.print(calibrated_min[count]);
      count++;
    }
    Serial.println("");
    Serial.println("------------------");
}
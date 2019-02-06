/*
 *
 */
#include "nimble.h"
#include "motor.h"
#include "nimble_control.h"
#include "nimble_display.h"

static nimble_state_t nimble_state;

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
            
    switch(nimble_state)
    {
        case STATE_SETUP:
            nimble_control_wait();
            nimble_go_to_state(STATE_CALIBRATION);
            break;
        case STATE_CALIBRATION:
            nimble_control_calibrate();
            nimble_print_calibrated_val();
//            nimble_go_to_state(STATE_GO);

//            nimble_debug_print("Press Button", eNIMBLE_LCD);
            //nimble_control_wait();

            a = (int)nimble_control_get_position();
 //           str = String(a);
 //           str.toCharArray(b,5);
//            nimble_debug_print(b, eNIMBLE_LCD);
//            delay(20);       
			
			if(a >= 1900 && a <= 2100)
			{
			  nimble_motor_setSpeeds(0,0);
              nimble_go_to_state(STATE_GO);
			}

//            nimble_motor_setSpeeds(255,240);
            break;
        case STATE_WAIT_START:
            break;
        case STATE_GO:
		while(1)
		{
			follow_segment();

            SetSpeeds(30, 30);
             delay(40);
         
             // These variables record whether the robot has seen a line to the
             // left, straight ahead, and right, whil examining the current
             // intersection.
             unsigned char found_left = 0;
             unsigned char found_straight = 0;
             unsigned char found_right = 0;

            a = (int)nimble_control_get_position();
		}


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
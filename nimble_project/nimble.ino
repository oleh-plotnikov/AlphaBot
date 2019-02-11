/*
 *
 */
#include "nimble.h"
#include "motor.h"
#include "nimble_control.h"
#include "nimble_display.h"

static nimble_state_t nimble_state;
unsigned int sensorValues[NUM_SENSORS];
unsigned int decision;

bool nimble_go_to_state(nimble_state_t state);
void nimble_print_calibrated_val();

void nimble_init()
{
    nimble_display_init();
    nimble_control_init();
    nimble_motor_init();
    decision = DECISION_LEFT;
    nimble_go_to_state(STATE_SETUP);
}

void nimble_run()
{
    int a; 
    char b[5];
    String str;
    unsigned int start_count = 5;
    static unsigned int calibrate_speed = 50;

    unsigned char found_left = 0;
    unsigned char found_straight = 0;
    unsigned char found_right = 0;
    unsigned char dir;        
    
    switch(nimble_state)
    {
        case STATE_SETUP:
//            if()
//            {
//                decision =
//            } 
//            else
//            {
//                decision =                
//            }
            
            nimble_go_to_state(STATE_CALIBRATION);
            break;
        case STATE_CALIBRATION:
            nimble_control_calibrate(calibrate_speed);
            nimble_print_calibrated_val();

            nimble_motor_setSpeeds(40, -40);
            
            for(unsigned int count = 0; count < 10000; count++)
            {
              a = (int)nimble_control_get_position(sensorValues);

              if(a >= 1995 && a <= 2010)
			  {
                nimble_motor_setSpeeds(-60, 60);
                delay(10);
                nimble_motor_setSpeeds(40, -40);
                
                a = (int)nimble_control_get_position(sensorValues);
                if(a >= 1990 && a <= 2010)
                {
                  nimble_motor_setSpeeds(0,0);
                  str = String(a);
                  str.toCharArray(b,5);
                  nimble_debug_print(b, eNIMBLE_LCD);
                  nimble_go_to_state(STATE_WAIT_START);
                }
              }
            }
            calibrate_speed -= 5;
            break;

        case STATE_WAIT_START:
            do{
              str = String(start_count);
              str.toCharArray(b,2);
              nimble_debug_print(b, eNIMBLE_LCD);
              delay(1000);
            }while(--start_count);
            nimble_go_to_state(STATE_GO);
            break;

        case STATE_GO:
//            nimble_debug_print("GO-GO-GO", eNIMBLE_LCD);
            follow_segment();
            nimble_motor_setSpeeds(30, 30);
            delay(40);

            found_left = 0;
            found_straight = 0;
            found_right = 0;
            nimble_go_to_state(STATE_MAKE_DECISION);
            break;

        case STATE_MAKE_DECISION:
            // Now read the sensors and check the intersection type.
           nimble_control_get_position(sensorValues);
        
            // Check for left and right exits.
            if (sensorValues[0] > 600)
            {
              found_left = 1;
//              nimble_debug_print("FOUND LEFT", eNIMBLE_LCD);
            }
            if (sensorValues[4] > 600)
            {
              found_right = 1;
 //             nimble_debug_print("FOUND RIGHT", eNIMBLE_LCD);
            }  
        
            // Drive straight a bit more - this is enough to line up our
            // wheels with the intersection.
            nimble_motor_setSpeeds(30, 30);
            delay(100);
        
            // Check for a straight exit.
            nimble_control_get_position(sensorValues);
            if (sensorValues[0] > 600 || sensorValues[2] > 600 || sensorValues[4] > 600)
            {
              found_straight = 1;
//              nimble_debug_print("FOUND STRAIGHT", eNIMBLE_LCD);
            }
        
            // Check for the ending spot.
            // If all three middle sensors are on dark black, we have
            // solved the maze.
            if (sensorValues[0] > 600 && sensorValues[2] > 600 && sensorValues[4] > 600)
            {
              nimble_motor_setSpeeds(0, 0);
              nimble_go_to_state(STATE_STOP);
              break;
            }
        
            dir = select_turn(found_left, found_straight, found_right, decision);
            turn(dir);
            nimble_go_to_state(STATE_GO);
            break;

        case STATE_STOP:
              nimble_debug_print("MAZE SOLVED", eNIMBLE_LCD);
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
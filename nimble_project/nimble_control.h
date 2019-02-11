#ifndef NIMBLE_CONTROL_H_
#define NIMBLE_CONTROL_H_

#include "TRSensors.h"

#define Addr  0x20
#define NUM_SENSORS 5

/**
 * @brief Decision type
 */
typedef enum
{
  DECISION_LEFT,
  DECISION_RIGHT
} decision_t;

void nimble_control_init();
void nimble_control_wait();
void nimble_control_calibrate(unsigned int speed);
unsigned int* nimble_control_get_min_ptr();
unsigned int* nimble_control_get_max_ptr();
unsigned int nimble_control_get_position(unsigned int *sensorValues);
void follow_segment();
unsigned char select_turn(unsigned char found_left, unsigned char found_straight, unsigned char found_right, decision_t decision);
void turn(unsigned char dir);

#endif
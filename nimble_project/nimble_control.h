#ifndef NIMBLE_CONTROL_H_
#define NIMBLE_CONTROL_H_

#include "TRSensors.h"

#define Addr  0x20
#define NUM_SENSORS 5

void nimble_control_init();
void nimble_control_wait();
void nimble_control_calibrate();
unsigned int* nimble_control_get_min_ptr();
unsigned int* nimble_control_get_max_ptr();
unsigned int nimble_control_get_position();
void follow_segment();

#endif
#ifndef NIMBLE_H_
#define NIMBLE_H_

/**
 * @brief Robot states
 */
typedef enum
{
  STATE_SETUP,
  STATE_CALIBRATION,
  STATE_WAIT_START,
  STATE_GO,
  STATE_STOP,
  STATE_MAKE_DECISION,
  STATE_FINISH
} nimble_state_t;

void nimble_init();
void nimble_run();

#endif
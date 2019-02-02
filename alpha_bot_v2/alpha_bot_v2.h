  #define DEBUG_MODE 1

  // Motor definitions
  #define PWMA   (6)   //Left Motor Speed pin (ENA)
  #define AIN2   (A0)  //Motor-L forward (IN2).
  #define AIN1   (A1)  //Motor-L backward (IN1)
  #define PWMB   (5)   //Right Motor Speed pin (ENB)
  #define BIN1   (A2)  //Motor-R forward (IN3)
  #define BIN2   (A3)  //Motor-R backward (IN4)

  // Line sensors definitions
  #define NUM_SENSORS   (5)

  // IO definitions
  #define IO_I2C_ADDR        (0x20)
  #define IO_MASK_BEEP_ON    (0xDF)
  #define IO_MASK_BEEP_OFF   (0x20)
  #define IO_MASK_JOY_UP     (0xFE)
  #define IO_MASK_JOY_DOWN   (0xF7)
  #define IO_MASK_JOY_LEFT   (0xFB)
  #define IO_MASK_JOY_RIGHT  (0xFD)
  #define IO_MASK_JOY_CENTER (0xEF)
  #define IO_MASK_DEFAULT    (0xE0)  //< Beep off, joystick released
  #define IO_MASK_RESET      (0xFF)  //< Reset IO state

  // Types declaration
  /**
   * @brief PID controller description
   */
  typedef struct PID_descr
  {
    const  int K_P;    // proportional gain
    const  int K_I;    // integral gain
    const  int K_D;    // derivative gain
    const  int OUT_MAX;  // maximal output
    long      integral; // integral value
    int       last_err; // last error value
  } PID_descr_t;

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
  } Robot_state_t;

  typedef enum
  {
    eCOMMAND_START,
  } Command_type_t;

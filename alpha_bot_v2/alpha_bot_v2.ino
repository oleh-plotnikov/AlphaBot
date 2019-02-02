#include <TRSensors.h>
#include <Wire.h>
#include "alpha_bot_v2.h"

TRSensors trs = TRSensors();
unsigned int line_sensor_buffer[NUM_SENSORS];
static PID_descr_t PID_gear = {20, 10000, 10, 80, 0, 0};
static Robot_state_t robot_state = STATE_SETUP;

char path[100] = "";
unsigned char path_length = 0; // the length of the path


// Support functions
void calibration(void);
void turn(unsigned char dir);
unsigned char select_turn(unsigned char found_left, unsigned char found_straight, unsigned char found_right);
void SetSpeeds(int left,int right);
void PCF8574Write(byte data);
byte PCF8574Read();
bool is_button_pressed(Command_type_t command);
void beep(unsigned long duration_ms);

// Loop functions
int PID_control(PID_descr_t* ptr_PID, int current, int target, bool reset);

// Setup board
void setup()
{
  delay(1000);
  Serial.begin(115200);
  Wire.begin();
  pinMode(PWMA,OUTPUT);                     
  pinMode(AIN2,OUTPUT);      
  pinMode(AIN1,OUTPUT);
  pinMode(PWMB,OUTPUT);       
  pinMode(AIN1,OUTPUT);     
  pinMode(AIN2,OUTPUT);  
  SetSpeeds(0,0);

  while(!is_button_pressed(eCOMMAND_START))
  {
    delay(10);
  }
  robot_state = STATE_SETUP;
  Serial.println("Bot is Ready to Start!!!");
  Serial.println("Send: ""start"" to calibrate");
}

// Main loop
void loop()
{
  switch (robot_state)
  {
    case STATE_SETUP:
    {
      if (wait_command("Waveshare"))
      {
        robot_state = STATE_CALIBRATION;
      }
      break;
    }
    case STATE_CALIBRATION:
    {
      delay(500);
      calibration();
      
      robot_state = STATE_WAIT_START;
      break;
    }
    case STATE_WAIT_START:
    {
      if (is_button_pressed(eCOMMAND_START))
      {
        // Reset current PID state and start to move
        PID_control(&PID_gear, 0, 0, true);
        delay(500);
        robot_state = STATE_GO;
      }
      break;
    }
    case STATE_GO:
    {
      follow_segment();
      robot_state = STATE_MAKE_DECISION;
      break;
    }
    case STATE_STOP:
    {
      break;
    }
    case STATE_MAKE_DECISION:
    {
      unsigned char found_left = 0;
      unsigned char found_straight = 0;
      unsigned char found_right = 0;
  
      // Now read the sensors and check the intersection type.
      trs.readLine(line_sensor_buffer);
  
      // Check for left and right exits.
      if (line_sensor_buffer[0] > 600)
        found_left = 1;
      if (line_sensor_buffer[4] > 600)
        found_right = 1;
  
      // Drive straight a bit more - this is enough to line up our
      // wheels with the intersection.
      SetSpeeds(20, 20);
      delay(100);
  
      // Check for a straight exit.
      trs.readLine(line_sensor_buffer);
      if (line_sensor_buffer[1] > 600 || line_sensor_buffer[2] > 600 || line_sensor_buffer[3] > 600)
        found_straight = 1;
  
      // Check for the ending spot.
      // If all three middle sensors are on dark black, we have
      // solved the maze.
      if (line_sensor_buffer[1] > 600 && line_sensor_buffer[2] > 600 && line_sensor_buffer[3] > 600)
      {
        SetSpeeds(0, 0);
		while(1)
	    {
			delay(1000);
		};
        break;
      }
  
      // Intersection identification is complete.
      // If the maze has been solved, we can follow the existing
      // path.  Otherwise, we need to learn the solution.
      unsigned char dir = select_turn(found_left, found_straight, found_right);
  
      // Make the turn indicated by the path.
      turn(dir);
      robot_state = STATE_GO;
      
	  path[path_length] = dir;
      path_length++;
	  
	  simplify_path();
      break;
    }
    default:
    {
      robot_state = STATE_SETUP;
      break;
    }
  }
}

void simplify_path()
{
  // only simplify the path if the second-to-last turn was a 'B'
  if (path_length < 3 || path[path_length-2] != 'B')
    return;

  int total_angle = 0;
  int i;
  for (i = 1; i <= 3; i++)
  {
    switch (path[path_length - i])
    {
    case 'R':
      total_angle += 90;
      break;
    case 'L':
      total_angle += 270;
      break;
    case 'B':
      total_angle += 180;
      break;
    }
  }

  // Get the angle as a number between 0 and 360 degrees.
  total_angle = total_angle % 360;

  // Replace all of those turns with a single one.
  switch (total_angle)
  {
  case 0:
    path[path_length - 3] = 'S';
    break;
  case 90:
    path[path_length - 3] = 'R';
    break;
  case 180:
    path[path_length - 3] = 'B';
    break;
  case 270:
    path[path_length - 3] = 'L';
    break;
  }

  // The path is now two steps shorter.
  path_length -= 2;
}

void print_sensor()
{
  trs.readLine(line_sensor_buffer);
  Serial.println(line_sensor_buffer[0]);
  Serial.println(line_sensor_buffer[1]);
  Serial.println(line_sensor_buffer[2]);
  Serial.println(line_sensor_buffer[3]);
  Serial.println(line_sensor_buffer[4]);
  delay(1000);
}

/**
 * @brief PID control implementation
 * 
 * @param ptr_PID .. pointer on PID settings
 * @param current .. current value
 * @param target  .. target value 
 * @param reset_I .. reset PID integrator
 * @return Output value of PID control
 */
int PID_control(PID_descr_t* ptr_PID, int current, int target, bool reset_I)
{
  int out = 0;
  // Input data verification
  if (NULL == ptr_PID)
  {
    return out;
  }
  // Reset PID state
  if (reset_I)
  {
    ptr_PID->integral = 0;
    ptr_PID->last_err = 0;
  }

  // PID step calculation
  int  P = 0;
  long I = ptr_PID->integral;
  int  D = 0;
  P = target - current;
  I += P;
  D = P - ptr_PID->last_err;
  // Out calculation
  out = P / ptr_PID->K_P + I / ptr_PID->K_I + D * ptr_PID->K_D;

  // Check limits
  if (out > ptr_PID->OUT_MAX)  out = ptr_PID->OUT_MAX;
  if (out < -ptr_PID->OUT_MAX) out = -ptr_PID->OUT_MAX;

  // Save PID state for next step
  ptr_PID->last_err = P;
  ptr_PID->integral = I;

  return out;
}

/**
 * @brief Setup TRS sensors
 */
void calibration(void)
{
  // @TODO: Rework calobration approach
  for (int i = 0; i < 100; i++)
  {
    if (i == 50 || i == 150)
	{
		SetSpeeds(0,0);
		delay(1000);
	}

    if (i < 25 || i > 75)
    {  
      SetSpeeds(60, -60);
    }
    else
    { 
      SetSpeeds(-60, 60);
    }
    trs.calibrate();
  }

  SetSpeeds(0, 0);
  delay(1000);
  SetSpeeds(-30, 30);
  int proportional = 1;
  line_sensor_buffer[2] = 100;
  while((proportional > 100 || proportional < -100))
  {
    unsigned int position = trs.readLine(line_sensor_buffer);
    proportional = ((int)position) - 2000;
  }
  SetSpeeds(0 , 0);
  Serial.println(proportional);
  Serial.println("------------------");
  Serial.println("Calibrated");
  
  uint8_t count = 0;
  Serial.print("MIN: ");
  while(count < NUM_SENSORS)
  {
    Serial.print(" ");
    Serial.print(trs.calibratedMin[count]);
    count++;
  }
  Serial.println("");

  count = 0;
  Serial.print("MAX: ");
  while(count < NUM_SENSORS)
  {
    Serial.print(" ");
    Serial.print(trs.calibratedMax[count]);
    count++;
  }
  Serial.println("");
  Serial.println("------------------");
}

// Code to perform various types of turns according to the parameter dir,
// which should be 'L' (left), 'R' (right), 'S' (straight), or 'B' (back).
// The delays here had to be calibrated for the 3pi's motors.
void turn(unsigned char dir)
{
  switch(dir)
  {
  case 'L':
    // Turn left.
    SetSpeeds(-100, 100);
    delay(190);
    break;
  case 'R':
    // Turn right.
    SetSpeeds(100, -100);
    delay(190);
    break;
  case 'B':
    // Turn around.
    SetSpeeds(100, -100);
    delay(400);
    break;
  case 'S':
    // Don't do anything!
    break;
  }
  SetSpeeds(0, 0);

  //Serial.write(dir);
  //Serial.println();
}

// This function decides which way to turn during the learning phase of
// maze solving.  It uses the variables found_left, found_straight, and
// found_right, which indicate whether there is an exit in each of the
// three directions, applying the "left hand on the wall" strategy.
unsigned char select_turn(unsigned char found_left, unsigned char found_straight, unsigned char found_right)
{
  // Make a decision about how to turn.  The following code
  // implements a left-hand-on-the-wall strategy, where we always
  // turn as far to the left as possible.
  if (found_left)
    return 'L';
  else if (found_straight)
    return 'S';
  else if (found_right)
    return 'R';
  else
    return 'B';
}

void SetSpeeds(int left,int right)
{
  if(left < 0)
  {
    digitalWrite(AIN1,HIGH);
    digitalWrite(AIN2,LOW);
    analogWrite(PWMA,-left);      
  }
  else
  {
    digitalWrite(AIN1,LOW); 
    digitalWrite(AIN2,HIGH);
    analogWrite(PWMA,left);  
  }
  
  if(right < 0)
  {
    digitalWrite(BIN1,HIGH);
    digitalWrite(BIN2,LOW);
    analogWrite(PWMB,-right);      
  }
  else
  {
    digitalWrite(BIN1,LOW); 
    digitalWrite(BIN2,HIGH);
    analogWrite(PWMB,right);  
  }
}

void PCF8574Write(byte data)
{
  Wire.beginTransmission(IO_I2C_ADDR);
  Wire.write(data);
  Wire.endTransmission(); 
}

byte PCF8574Read()
{
  int data = -1;
  Wire.requestFrom(IO_I2C_ADDR, 1);
  if(Wire.available()) {
    data = Wire.read();
  }
  return static_cast<byte>(data);
}

bool is_button_pressed(Command_type_t command)
{
  byte button_mask = IO_MASK_JOY_CENTER;
  switch (command)
  {
    case eCOMMAND_START:
      button_mask = IO_MASK_JOY_CENTER;
      break; 
/*
    case COMMAND_:
      byte button_mask = IO_MASK_JOY_DOWN;
      break; 
    case COMMAND_:
      byte button_mask = IO_MASK_JOY_UP;
      break; 
    case COMMAND_:
      byte button_mask = IO_MASK_JOY_LEFT;
      break; 
    case COMMAND_:
      byte button_mask = IO_MASK_JOY_RIGHT;
      break;
*/
    default:
      button_mask = IO_MASK_JOY_CENTER;
      break;
  }
  
#if DEBUG_MODE
  byte reg_value = IO_MASK_RESET;
  PCF8574Write(reg_value);
  reg_value = PCF8574Read() | IO_MASK_DEFAULT;
  return reg_value == button_mask;
#else
  String comdata = "";
  while (Serial.available() > 0)  
  {
    comdata += char(Serial.read());
    delay(2);
  }
  if (comdata.length() > 0)
  {
    Serial.println(comdata);
    const char* command = comdata.c_str();
    if(strcmp(command, str) == 0)          //Forward
    {
      return 1;
    }
  }
  comdata = "";
  return 0;
#endif 
}

void beep(unsigned long duration_ms)
{
  PCF8574Write(IO_MASK_BEEP_ON);
  delay(duration_ms);
  PCF8574Write(IO_MASK_BEEP_OFF);
}

unsigned long lasttime = 0;
void follow_segment()
{
  int last_proportional = 0;
  long integral=0;

  while(1)
  {
    unsigned int position = trs.readLine(line_sensor_buffer);

    int proportional = ((int)position) - 2000;

    int derivative = proportional - last_proportional;
    integral += proportional;

    last_proportional = proportional;

    int power_difference = proportional/20 + integral/10000 + derivative*10;

    const int maximum = 150; // the maximum speed
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

   if(millis() - lasttime >100)
   {
    if (line_sensor_buffer[1] < 150 && line_sensor_buffer[2] < 150 && line_sensor_buffer[3] < 150)
    {
//      robot_state = STATE_WAIT_START;
//	  while(1)/
//	  {
//          SetSpeeds(0, 0);
//		  delay(1000);
//	  }
      return;
    }
    else if (line_sensor_buffer[0] > 600 || line_sensor_buffer[4] > 600)
    {
      // Found an intersection.
      SetSpeeds(0, 0);
      return;
    }
   }
  }
}

bool wait_command(const char* str)
{
  String comdata = "";
  while (Serial.available() > 0)  
  {
    comdata += char(Serial.read());
    delay(2);
  }
  if (comdata.length() > 0)
  {
    Serial.println(comdata);
    const char* command = comdata.c_str();
    if(strcmp(command, str) == 0)          //Forward
    {
      return 1;
    }
  }
  comdata = "";
  return 0;
}
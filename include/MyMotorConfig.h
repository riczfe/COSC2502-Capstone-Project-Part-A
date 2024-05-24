#include <Arduino.h>

#define MOT_AIN1 14
#define MOT_BIN1 27
#define MOT_CIN1 26
#define MOT_DIN1 25

#define PWM_CHA_AIN1 0
#define PWM_CHA_BIN1 2
#define PWM_CHA_CIN1 4
#define PWM_CHA_DIN1 6


#define PWM_FREQ 30000
#define PWM_RES 8

extern double motor_cmd_x, motor_cmd_y; 
int PWM_A = 127, PWM_B = 127, PWM_C = 127, PWM_D = 127;
// int LValue, RValue, commaIndex;

void Init_MotorPin()
{
  // Define GPIO as output
  pinMode(MOT_AIN1, OUTPUT);
  pinMode(MOT_BIN1, OUTPUT);
  pinMode(MOT_CIN1, OUTPUT);
  pinMode(MOT_DIN1, OUTPUT);

  // Configure PWM channel with PWM frequency and resolution
  ledcSetup(PWM_CHA_AIN1, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHA_BIN1, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHA_CIN1, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHA_DIN1, PWM_FREQ, PWM_RES);


  // Attach PWM channel to GPIO
  ledcAttachPin(MOT_AIN1, PWM_CHA_AIN1);
  ledcAttachPin(MOT_BIN1, PWM_CHA_BIN1);
  ledcAttachPin(MOT_CIN1, PWM_CHA_CIN1);
  ledcAttachPin(MOT_DIN1, PWM_CHA_DIN1);

}

void PID_X()
{
  ledcWrite(PWM_CHA_AIN1, motor_cmd_x);
  ledcWrite(PWM_CHA_BIN1, motor_cmd_x);
}

void PID_Y()
{
  ledcWrite(PWM_CHA_AIN1, motor_cmd_y);
  ledcWrite(PWM_CHA_DIN1, motor_cmd_y);
}
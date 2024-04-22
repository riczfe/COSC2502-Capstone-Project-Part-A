#include <Arduino.h>

#define MOT_ENA1 23
#define MOT_ENA2 32
#define MOT_AIN1 22
#define MOT_AIN2 21
#define MOT_BIN1 18
#define MOT_BIN2 19
#define MOT_CIN1 33
#define MOT_CIN2 25
#define MOT_DIN1 27
#define MOT_DIN2 26
#define PWM_CHA_AIN1 0
#define PWM_CHA_AIN2 1
#define PWM_CHA_BIN1 2
#define PWM_CHA_BIN2 3
#define PWM_CHA_CIN1 4
#define PWM_CHA_CIN2 5
#define PWM_CHA_DIN1 6
#define PWM_CHA_DIN2 7

#define PWM_FREQ 30000
#define PWM_RES 8

extern double motor_cmd; 
int PWM_A = 127, PWM_B = 127, PWM_C = 127, PWM_D = 127;
int LValue, RValue, commaIndex;

void Init_MotorPin()
{
  // Define GPIO as output
  pinMode(MOT_ENA1, OUTPUT);
  pinMode(MOT_ENA2, OUTPUT);
  pinMode(MOT_AIN1, OUTPUT);
  pinMode(MOT_AIN2, OUTPUT);
  pinMode(MOT_BIN1, OUTPUT);
  pinMode(MOT_BIN2, OUTPUT);
  pinMode(MOT_CIN1, OUTPUT);
  pinMode(MOT_CIN2, OUTPUT);
  pinMode(MOT_DIN1, OUTPUT);
  pinMode(MOT_DIN2, OUTPUT);

  // Configure PWM channel with PWM frequency and resolution
  ledcSetup(PWM_CHA_AIN1, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHA_AIN2, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHA_BIN1, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHA_BIN2, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHA_CIN1, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHA_CIN2, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHA_DIN1, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHA_DIN2, PWM_FREQ, PWM_RES);

  // Attach PWM channel to GPIO
  ledcAttachPin(MOT_AIN1, PWM_CHA_AIN1);
  ledcAttachPin(MOT_AIN2, PWM_CHA_AIN2);
  ledcAttachPin(MOT_BIN1, PWM_CHA_BIN1);
  ledcAttachPin(MOT_BIN2, PWM_CHA_BIN2);
  ledcAttachPin(MOT_CIN1, PWM_CHA_CIN1);
  ledcAttachPin(MOT_CIN2, PWM_CHA_CIN2);
  ledcAttachPin(MOT_DIN1, PWM_CHA_DIN1);
  ledcAttachPin(MOT_DIN2, PWM_CHA_DIN2);

  // invert the output of GPIO
  GPIO.func_out_sel_cfg[MOT_AIN1].inv_sel = 1; 
  GPIO.func_out_sel_cfg[MOT_BIN2].inv_sel = 1; 
  GPIO.func_out_sel_cfg[MOT_CIN2].inv_sel = 1; 
  GPIO.func_out_sel_cfg[MOT_DIN2].inv_sel = 1; 

  // Motor enable
  digitalWrite(MOT_ENA1, HIGH);
  digitalWrite(MOT_ENA2, HIGH);
  PWM_A = 127; 
  PWM_C = 127; 
  ledcWrite(PWM_CHA_AIN1, PWM_A);
  ledcWrite(PWM_CHA_AIN2, PWM_A);
  ledcWrite(PWM_CHA_CIN1, PWM_C);
  ledcWrite(PWM_CHA_CIN2, PWM_C);
}

void Run_Motor()
{
  ledcWrite(PWM_CHA_AIN1, motor_cmd);
  ledcWrite(PWM_CHA_AIN2, motor_cmd);
  ledcWrite(PWM_CHA_CIN1, motor_cmd);
  ledcWrite(PWM_CHA_CIN2, motor_cmd);
}

#include <Arduino.h>
#include <PID_v1.h>

int Fall_Dectect = 0;
extern double anglex;
extern double angley;

// ================================================================
// Variable declaration
// ================================================================
// The PID object is configured as follows:
// input = sensor, variable to be controller;
// output = pid output, command sent to the motors;
// setpoint = reference setpoint, the desired angle (usually 0deg to maintain an upward position)
// PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

double pid_output_x = 0, motor_cmd_x = 127, motor_cmd_y = 127;
double pid_output_y = 0;
// Init gain
double kp = 12.0, ki = 100.0, kd = 0.15, anglex_setpoint = 0, angley_setpoint = 0;
// Correct gain 
// double kp = 12.0, ki = 100.0, kd = 0.15, anglex_setpoint = 1;

PID myPIDforX(&anglex, &pid_output_x, &anglex_setpoint, kp, ki, kd, DIRECT);
PID myPIDforY(&angley, &pid_output_y, &angley_setpoint, kp, ki, kd, DIRECT);

// ================================================================
// Function Definition
// ================================================================
void Init_PID()
{
  myPIDforX.SetMode(AUTOMATIC);
  myPIDforX.SetOutputLimits(-127, 127);
  myPIDforX.SetSampleTime(10);
}
// ================================================================
void Compute_PID()
{
  // X angle tunning
  myPIDforX.SetTunings(kp, ki, kd);
  myPIDforX.Compute();
  // if (abs(anglex) > 30) {
  //   pid_output = 0; // motor stop when fall
  //   Fall_Dectect = 1;
  // }
  motor_cmd_x = map(pid_output_x, -127, 127, 0, 255); 

  // Y angle tunning
  myPIDforY.SetTunings(kp, ki, kd);
  myPIDforY.Compute();
  // if (abs(angley) > 30) {
  //   pid_output = 0; // motor stop when fall
  //   Fall_Dectect = 1;
  // }
  motor_cmd_y = map(pid_output_x, -127, 127, 0, 255);
}
// ================================================================
#include <Arduino.h>
#include <PID_v1.h>

extern double anglex;
// ================================================================
// Variable declaration
// ================================================================
// The PID object is configured as follows:
// input = sensor, variable to be controller;
// output = pid output, command sent to the motors;
// setpoint = reference setpoint, the desired angle (usually 0deg to maintain an upward position)
// PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

double pid_output = 0, motor_cmd = 127;
// Init gain
double kp = 5.0, ki = 0.0, kd = 0.0, anglex_setpoint = 0;
// Correct gain 
// double kp = 12.0, ki = 100.0, kd = 0.15, anglex_setpoint = 1;

PID myPID(&anglex, &pid_output, &anglex_setpoint, kp, ki, kd, DIRECT);

// ================================================================
// Function Definition
// ================================================================
void Init_PID()
{
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-127, 127);
  myPID.SetSampleTime(10);
}
// ================================================================
void Compute_PID()
{
  myPID.SetTunings(kp, ki, kd);
  myPID.Compute();
  if (abs(anglex) > 30)
    pid_output = 0; // motor stop when fall
  motor_cmd = map(pid_output, -127, 127, 0, 255); 
}
// ================================================================

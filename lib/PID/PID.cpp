/**
 * @file pid.cpp
 * @author 
 * @brief 
 * @version 0.1
 * @date 22-04-2024
 */

#include "PID.h"


/**
 * Initializes a PID controller structure with specified control parameters and initial values.
 * 
 * @param pid Pointer to a PID structure to initialize.
 * @param setpoint The desired target value the PID controller will try to achieve.
 * @param Kp Proportional gain, determines the reaction to the current error.
 * @param Ki Integral gain, determines the reaction based on the sum of recent errors.
 * @param Kd Derivative gain, determines the reaction to the rate of change of the error.
 */
void PID_init(PID* pid, float setpoint, float Kp, float Ki, float Kd){
    pid->setpoint = setpoint;                // Set the target setpoint.
    pid->Kp = Kp;                            // Set proportional gain
    pid->Ki = Ki;                            // Set integral gain.
    pid->Kd = Kd;                            // Set derivative gain.
    pid->dt = 0;                             // Initialize time difference to zero.
    pid->err[0] = pid->err[1] = 0;           // Initialize error terms.
    pid->output[0] = pid->output[1] = 0;     // Initialize output terms.
    pid->min = PID_MIN;                      // Set minimum output limit.
    pid->max = PID_MAX;                      // Set maximum output limit.
}

/**
 * Updates the PID controller with a new sample and computes the PID control output.
 * 
 * @param pid Pointer to the PID structure.
 * @param sample The latest measurement from the sensor.
 */
void PID_update(PID* pid, float sample){

    // Shift the previous errors.
    pid->err[1] = pid->err[0];
    // Compute the new error.
    pid->err[0] = pid->setpoint - sample;

    // Update integral.
    pid->integral += pid->err[0]*pid->dt;

    // Compute each term of the PID control.
    pid->P = pid->Kp * pid->err[0];         // Proportional term.
    pid->I = pid->Ki * pid->integral;       // Integral term.
    pid->D = pid->Kd * (sample/pid->dt);   // Derivative term, corrected to use error difference.
   
    // Compute total output.
    pid->output[1] = pid->output[0];
    pid->output[0] = pid->P + pid->I + pid->D;

    // Apply saturation filter to limit the PID output.
    if (pid->output[0] > pid->max) pid->output[0] = pid->max;
    else if (pid->output[0] < pid->min) pid->output[0] = pid->min;
}

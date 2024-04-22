/**
 * @file pid.h
 * Defines the structure and function prototypes for a PID (Proportional, Integral, Derivative) controller.
 * PID controllers are widely used in control systems to adjust the output based on the error between a desired setpoint and a measured process variable.
 * 
 * @author [Your Name]
 * @version 0.1
 * @date 22-04-2024
 */

#pragma once
#ifndef PID_H
#define PID_H

#include <stdint.h>
#include "Arduino.h"
#include "MPU9250.h"

// Saturation limits for PID output.
#define PID_MIN -999  
#define PID_MAX 999
#define X_SETPOINT 500
#define Y_SETPOINT 500

/**
 * PID controller structure.
 */
typedef struct PID {
    float setpoint;     // Target setpoint.
    float Kp, Ki, Kd;   // PID gains for proportional, integral, and derivative terms.
    float dt;           // Time interval in seconds.
    float err[2];       // Error terms, where err[0] is the current error and err[1] is the previous error.
    float output[2];    // Output terms, where output[0] is the current output and output[1] is the previous output.
    float P, I, D;      // Individual components of the PID output.
    float integral;     // Accumulated integral term.
    float min, max;     // Minimum and maximum output limits.
} PID;


// Function declarations for initializing and updating PID controllers.
void PID_init(PID* pid, float setpoint, float Kp, float Ki, float Kd);
void PID_update(PID* pid, float sample);

#endif
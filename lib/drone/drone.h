/**
 * @file drone.h
 * @author 
 * @brief Drone essential structure and functions. 
 * @date 22-04-2024
 */

#pragma once
#ifndef DRONE_H
#define DRONE_H

#include <stdint.h>
#include "PID.h"
#include "MPU9250.h"

// Constants for drone operation
#define N 5  // Number of samples for signal averaging
#define ESC_ARM_SIGNAL 1000
#define ESC_ARM_TIME 7000
#define MIN_PWM_VALUE 1000
#define MAX_PWM_VALUE 1980
#define FLUCTUATION_TOLERANCE 9
#define VARIATION_TOLERANCE 200
#define AV_TOLERANCE 200
#define CONTROLLABILITY_THRESHOLD 1080 // Minimum throttle value for effective control


/**
 * @brief contains essential paramenters for a quadcopter drone
 * Structure to hold all necessary drone parameters and state.
 */
typedef struct drone{
    uint8_t state; // Current state of the drone (disarmed, armed, active control)

    int16_t av_throttle;        // Averaged throttle signal
    int16_t av_roll;            // Averaged roll signal
    int16_t av_pitch;           // Averaged pitch signal
    int16_t av_yaw;             // Averaged yaw signal

    int16_t throttle[N];        // History of throttle signals
    int16_t roll[N];            // History of roll signals
    int16_t pitch[N];           // History of pitch signals
    int16_t yaw[N];             // History of yaw signals
    int16_t ch5;

    float roll_coeff;           // Roll sensitivity coefficient
    float pitch_coeff;          // Pitch sensitivity coefficient
    float yaw_coeff;            // Yaw sensitivity coefficient

    // Motor control signals
    uint16_t m1;
    uint16_t m2;            
    uint16_t m3;
    uint16_t m4;

    IMU* imu;   // IMU data for control
    PID* pid_x; // PID controller for x-axis (roll)  - pid control on gyro_x
    PID* pid_y; // PID controller for y-axis (pitch) - pid control on gyro_y
} drone;

// Drone states enumeration
enum state{
    disarmed = 0, 
    armed = 1,
    active_control = 2
};

// Function prototypes
uint8_t drone_init(drone* d, float r_coeff, float p_coeff, float y_coeff, float Kp, float Ki, float Kd);
uint8_t compute_motor_signal(drone* d);
inline int16_t signal_average(int16_t* a);
uint8_t update_needed(drone* d, drone* prev_d);
void compute_control_action(drone* d);


#endif
/**
 * @file drone.cpp
 * @author 
 * @brief 
 * @version 0.1
 * @date 22-04-2024
 */


#include <stdio.h>
#include <string.h>
#include "drone.h"

/**
 * @brief Initialize drone structure
 * 
 * @param d pointer to a drone structure
 * @param r_coeff roll sensitivity coefficient
 * @param p_coeff pitch sensitivity coefficient
 * @param y_coeff yaw sensitivity coefficient
 * @param Kp Proportional gain
 * @param Ki Integral gain
 * @param Kd Derivative gain
 * @return uint8_t 
 */

uint8_t drone_init(drone* d, float r_coeff, float p_coeff, float y_coeff, float Kp, float Ki, float Kd){
  d->state = disarmed;
  
  int16_t tmp[N] = {MIN_PWM_VALUE+1};     // Initialize all control signals just above minimum PWM value.
  memcpy(d->throttle, tmp, sizeof(tmp));
  memcpy(d->roll, tmp, sizeof(tmp));
  memcpy(d->pitch, tmp, sizeof(tmp));
  memcpy(d->yaw, tmp, sizeof(tmp));

  // Calculate average signals initially to stabilize the output from start.
  d->av_throttle  = signal_average(d->throttle);
  d->av_roll      = signal_average(d->roll);
  d->av_pitch     = signal_average(d->pitch);
  d->av_yaw       = signal_average(d->yaw);

  // Set all motors to 0 to ensure no unintended movement.
  d->m1 = d->m2 = d->m3 = d->m4 = 0;

  // Assign sensitivity coefficients for each control axis.
  d->roll_coeff   = r_coeff;
  d->pitch_coeff  = p_coeff;
  d->yaw_coeff    = y_coeff;

  // Initialize PID controllers for roll and pitch.
  PID_init(d->pid_x, X_SETPOINT, Kp, Ki, Kd);
  PID_init(d->pid_y, Y_SETPOINT, Kp, Ki, Kd);

  // Reset the time delta for PID calculations.
  d->imu->dt = 0;
  return d->state;
}

/**
 * @brief Mapping: Throttle,Roll,Pitch,Yaw --> m1,m2,m3,m4
 * 
 * @param d pointer to a drone structure
 * @return 1 if update needed, 0 if not
 */
/**
 * Computes motor signals based on the current throttle, roll, pitch, and yaw values.
 * It also applies saturation to ensure motor signals stay within allowable PWM values.
 * 
 * @param d Pointer to a drone structure.
 * @return 1 if motor signals were updated, otherwise 0.
 */
uint8_t compute_motor_signal(drone* d){
  d->av_throttle = signal_average(d->throttle);
  d->av_roll = signal_average(d->roll);
  d->av_pitch = signal_average(d->pitch);
  d->av_yaw = signal_average(d->yaw);

  // Recalculate average control signals.
  if(d->av_throttle < CONTROLLABILITY_THRESHOLD){
    d->m1 = d->m2 = d->m3 = d->m4 = d->av_throttle;
    return 1; // Signal that an update is needed.
  }

  // Adjust control signals around center (1500) and compute motor outputs based on control coefficients.
  d->av_roll -= 1500;
  d->av_pitch -= 1500;
  d->av_yaw -= 1500;
  d->m1 = d->av_throttle + d->roll_coeff*d->av_roll - d->pitch_coeff*d->av_pitch + d->yaw_coeff*d->av_yaw;
  d->m2 = d->av_throttle - d->roll_coeff*d->av_roll - d->pitch_coeff*d->av_pitch - d->yaw_coeff*d->av_yaw;
  d->m3 = d->av_throttle - d->roll_coeff*d->av_roll + d->pitch_coeff*d->av_pitch + d->yaw_coeff*d->av_yaw;
  d->m4 = d->av_throttle + d->roll_coeff*d->av_roll + d->pitch_coeff*d->av_pitch - d->yaw_coeff*d->av_yaw;
  d->av_roll += 1500;
  d->av_pitch += 1500;
  d->av_yaw += 1500;

  // saturation filter
  if (d->m1 < 1012) d->m1 = MIN_PWM_VALUE;
  else if (d->m1 > MAX_PWM_VALUE) d->m1 = MAX_PWM_VALUE;
  
  if (d->m2 < 1012) d->m2 = MIN_PWM_VALUE;
  else if (d->m2 > MAX_PWM_VALUE) d->m2 = MAX_PWM_VALUE;
  
  if (d->m3 < 1012) d->m3 = MIN_PWM_VALUE;
  else if (d->m3 > MAX_PWM_VALUE) d->m3 = MAX_PWM_VALUE;
  
  if (d->m4 < 1012) d->m4 = MIN_PWM_VALUE;
  else if (d->m4 > MAX_PWM_VALUE) d->m4 = MAX_PWM_VALUE;

/*
  // motor average
  uint16_t m_av = (d->m1 + d->m2 + d->m3 + d->m4)/4;
  float m_weight = m_av/1500;

  // averaging filter
  if(d->m1 > m_av+AV_TOLERANCE) d->m1 = m_av+AV_TOLERANCE;
  else if(d->m1 < m_av-(AV_TOLERANCE*m_weight)) d->m1 = m_av-(AV_TOLERANCE*m_weight);

  if(d->m2 > m_av+(AV_TOLERANCE*m_weight)) d->m2 = m_av+(AV_TOLERANCE*m_weight);
  else if(d->m2 < m_av-(AV_TOLERANCE*m_weight)) d->m2 = m_av-(AV_TOLERANCE*m_weight);

  if(d->m3 > m_av+(AV_TOLERANCE*m_weight)) d->m3 = m_av+(AV_TOLERANCE*m_weight);
  else if(d->m3 < m_av-(AV_TOLERANCE*m_weight)) d->m3 = m_av-(AV_TOLERANCE*m_weight);

  if(d->m4 > m_av+(AV_TOLERANCE*m_weight)) d->m4 = m_av+AV_TOLERANCE;
  else if(d->m4 < m_av-(AV_TOLERANCE*m_weight)) d->m4 = m_av-(AV_TOLERANCE*m_weight);
*/
  return 1; // Always for safety
  //return 0; // to not update
}

/**
 * Computes the average of a signal array, typically used for smoothing out control signals.
 * 
 * @param a Pointer to an array of signals.
 * @return The average value of the signal array.
 */
inline int16_t signal_average(int16_t* a){
  return (a[0]+a[1]+a[2]+a[3]+a[4])/N;
}

/*
uint8_t update_needed(drone* d){ 
  if( (d->throttle[0] - d->throttle[1] < -FLUCTUATION_TOLERANCE)||
    (d->throttle[0] - d->throttle[1] > FLUCTUATION_TOLERANCE)||
    (d->roll[0] - d->roll[1] < -FLUCTUATION_TOLERANCE)||
    (d->roll[0] - d->roll[1] > FLUCTUATION_TOLERANCE)||
    (d->pitch[0] - d->pitch[1] < -FLUCTUATION_TOLERANCE)||
    (d->pitch[0] - d->pitch[1] > FLUCTUATION_TOLERANCE)||
    (d->yaw[0] - d->yaw[1] < -FLUCTUATION_TOLERANCE)||
    (d->yaw[0] - d->yaw[1] > FLUCTUATION_TOLERANCE)){
      //memcpy(d, prev_d, sizeof(drone)); // discard the actual sample
      return 0;
  }
    else{
      return 1;
  }
}
*/

/**
 * @brief Compute a PID control action based on gyro data
 * 
 * @param d pointer to a drone structure
 */
/**
 * Computes a PID control action based on the gyro data for roll and pitch.
 * 
 * @param d Pointer to a drone structure.
 */
void compute_control_action(drone* d){

  // Update the PID controllers' time delta.
  d->pid_x->dt = d->pid_y->dt = d->imu->dt;

  // Update the PID controller for roll using x-axis gyro data.
  PID_update(d->pid_x, d->imu->gyro_x);

  // Update the PID controller for pitch using y-axis gyro data.
  PID_update(d->pid_y, d->imu->gyro_y);

}
#include <Arduino.h>       // Arduino library
#include "MyMPU.h"         // Personal library to configure the MPU6050
#include "MySerial.h"      // Personal library to configure the serial communication
#include "MyMotorConfig.h" // Personal library to configure the motor
#include "MyPID.h"         // Personnal library to configure the PID

// ================================================================
// Variable declaration
// ================================================================
// Most of the variables are declared in the personal library
// ================================================================
// Function Declaration
// ================================================================
// These function are kept in the main.cpp because it is easier to modify
void SerialDataPrint(); // Data from the microcontroller to the PC
void SerialDataWrite(); // Data from the PC to the microcontroller
// ================================================================
// Setup function
// ================================================================
void setup()
{
  Init_Serial();   // Initialize the serial communication
  Init_MPU();      // Initialize the MPU
  Init_MotorPin(); // Initialize the motor pin
  Init_PID();      // Initialize the PID
}
// ================================================================
// Loop function
// ================================================================
void loop()
{
  Get_MPUangle();    // Get the angle (anglex) from the IMU sensor
  Compute_PID();     // Compute the PID output (motor_cmd)
  Run_Motor();       // Send the PID output (motor_cmd) to the motor
  SerialDataPrint(); // Print the data on the serial monitor for debugging
  SerialDataWrite(); // User data to tune the PID parameters
}

// ================================================================
// Function Definition
// ================================================================
void SerialDataPrint()
{
  if (micros() - time_prev >= 50000)
  {
    time_prev = micros();
    Serial.print(millis());
    Serial.print("\t");
    Serial.print(anglex);
    Serial.print("\t");
    Serial.print(motor_cmd);
    Serial.print("\t");
    Serial.print(kp);
    Serial.print("\t");
    Serial.print(ki);
    Serial.print("\t");
    Serial.print(kd);
    Serial.print("\t");
    Serial.print(anglex_setpoint);

    Serial.println();
  }
}

// ================================================================
// Function to tune the PID parameters. For example: 
// To change the P value to 10, type p10
// To change the I value to -5, type i-5
// To change the D value to 2.4, type d2.4
// To change the setpoint to 3, type s3

void SerialDataWrite()
{
  static String received_chars;
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    received_chars += inChar;
    if (inChar == '\n')
    {
      switch (received_chars[0])
      {
      case 'p':
        received_chars.remove(0, 1);
        kp = received_chars.toFloat();
        break;
      case 'i':
        received_chars.remove(0, 1);
        ki = received_chars.toFloat();
        break;
      case 'd':
        received_chars.remove(0, 1);
        kd = received_chars.toFloat();
        break;
      case 's':
        received_chars.remove(0, 1);
        anglex_setpoint = received_chars.toFloat();
      default:
        break;
      }
      received_chars = "";
    }
  }
}

#include <Arduino.h>       // Arduino library
#include "MyMPU.h"         // Personal library to configure the MPU6050
#include "MySerial.h"      // Personal library to configure the serial communication
#include "MyMotorConfig.h" // Personal library to configure the motor
#include "MyPID.h"         // Personnal library to configure the PID
#include <ESP32Servo.h>


#define MAX_SIGNAL 2000  // Maximum PWM signal for ESC
#define MIN_SIGNAL 1000  // Minimum PWM signal for ESC
#define POT_PIN 4        // Pin attached to the potentiometer

Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;                      // Define the ESC
int CtrlPWM;                    // Control Signal for ESC (0 - 180 range)


void Init_ESC();                // Function to init the ESC
void WaitForKeyStroke(); // Function to interact with the serial monitor

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
  Init_MotorPin(); // Initialize the motor pin
  Init_ESC();                 // Initialize the ESC
  Init_MPU();      // Initialize the MPU
  Init_PID();      // Initialize the PID
}
// ================================================================
// Loop function
// ================================================================
void loop()
{

  CtrlPWM = map(analogRead(POT_PIN), 0, 4095, 1000, 2000); // Read the pot, map the reading from [0, 4095] to [0, 180]
  
  // Get data from MPU6050
  Get_MPUangle();
  Get_accelgyro();
  // Apply tunning
  Compute_PID();     // Compute the PID output for x and y angle

  if(CtrlPWM >= 1100 && CtrlPWM <= 1900){  
    ESC3.write(CtrlPWM + motor_cmd_x + motor_cmd_y);
    ESC4.write(CtrlPWM + motor_cmd_x);  
    ESC2.write(CtrlPWM + motor_cmd_y);
  }else{
    ESC1.write(CtrlPWM);
    ESC2.write(CtrlPWM);
    ESC3.write(CtrlPWM);
    ESC4.write(CtrlPWM);

  }


  //   SerialDataWrite(); // User data to tune the PID parameters
  SerialDataPrint(); 

  
}



// ================================================================
// Function Definition
// ================================================================

void Init_ESC() {
    ESC1.attach(MOT_AIN1, MIN_SIGNAL, MAX_SIGNAL);
    ESC2.attach(MOT_BIN1, MIN_SIGNAL, MAX_SIGNAL);
    ESC3.attach(MOT_CIN1, MIN_SIGNAL, MAX_SIGNAL);
    ESC4.attach(MOT_DIN1, MIN_SIGNAL, MAX_SIGNAL);
    ESC1.writeMicroseconds(MIN_SIGNAL);
    ESC2.writeMicroseconds(MIN_SIGNAL);
    ESC3.writeMicroseconds(MIN_SIGNAL);
    ESC4.writeMicroseconds(MIN_SIGNAL);
    Serial.println("ESC Initialization Complete: Minimum signal sent.");

    Serial.println();
    Serial.println("Calibration step 1. Disconnect the battery.");
    Serial.println("Press any key to continue.");
    WaitForKeyStroke();
    ESC1.writeMicroseconds(MAX_SIGNAL); // Sending MAX_SIGNAL tells the ESC to enter calibration mode
    ESC2.writeMicroseconds(MAX_SIGNAL);
    ESC3.writeMicroseconds(MAX_SIGNAL);
    ESC4.writeMicroseconds(MAX_SIGNAL);


    Serial.println();
    Serial.println("Calibration step 2. Connect the battery.");
    Serial.println("Wait for two short bips.");
    Serial.println("Press any key to continue.");
    WaitForKeyStroke();

    ESC1.writeMicroseconds(MIN_SIGNAL); // Sending MIN_SIGNAL tells the ESC the calibration value
    ESC2.writeMicroseconds(MIN_SIGNAL); 
    ESC3.writeMicroseconds(MIN_SIGNAL); 
    ESC4.writeMicroseconds(MIN_SIGNAL); 
    Serial.println();
    Serial.println("Wait for 4 short bips, and one long bip.");
    Serial.println("Press any key to finish.");
    WaitForKeyStroke();
}


void WaitForKeyStroke()
{
  while (!Serial.available())
    ;
  while (Serial.available())
    Serial.read();
}

void SerialDataPrint()
{
  if (micros() - time_prev >= 50000)
  {
    if (millis() - time_prev >= 50) {  // Print every second
        time_prev = millis();
        Serial.print("\n");
        Serial.print("Time(ms): ");
        Serial.print(time_prev);
        Serial.print("   ESC Signal: ");
        Serial.print(CtrlPWM);
        Serial.print("\t");
        Serial.print(anglex);
        Serial.print("\t");
        Serial.print(angley);
        Serial.print("\t");
        Serial.print(anglez);
        Serial.print("\t");
        Serial.print(gyrox);
        Serial.print("\t");
        Serial.print(gyroy);
        Serial.print("\t");
        Serial.print(gyroz);
        Serial.print("      \t");
        Serial.print(motor_cmd_x);
        Serial.print("\t");
        Serial.print(motor_cmd_y);



    }

    // time_prev = micros();
    // Serial.print(millis());
    // Serial.print("\t");
    // Serial.print(anglex);
    // Serial.print("\t");
    // Serial.print(motor_cmd);
    // Serial.print("\t");
    // Serial.print(kp);
    // Serial.print("\t");
    // Serial.print(ki);
    // Serial.print("\t");
    // Serial.print(kd);
    // Serial.print("\t");
    // Serial.print(anglex_setpoint);
    // Serial.print("okayy");

    // Serial.println();
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
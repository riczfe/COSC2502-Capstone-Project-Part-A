#include <Arduino.h>
#include <ESP32Servo.h>
// ================================================================
// Variable declaration
// ================================================================
#define MAX_SIGNAL 2000 // Parameter required for the ESC definition
#define MIN_SIGNAL 1000 // Parameter required for the ESC definition
#define MOTOR_PIN 13    // Pin 13 attached to the ESC signal pin
#define POT_PIN 4       // Pin 4 attached to the potentiometer

Servo ESC;                   // Define the ESC
int CtrlPWM;                 // Control Signal. Varies between [0 - 180]
unsigned long time_prev = 0; // Variable used for serial monitoring
// ================================================================
// Function declaration
// ================================================================
void SerialDataPrint();  // Function to print data on the serial monitor
void Init_Serial();      // Function to init the serial monitor
void WaitForKeyStroke(); // Function to interact with the serial monitor

// ================================================================
// Setup
// ================================================================
void setup()
{
  Init_Serial();                                 // Initialize the serial communication
  ESC.attach(MOTOR_PIN, MIN_SIGNAL, MAX_SIGNAL); // Initialize the ESC

  Serial.println();
  Serial.println("Calibration step 1. Disconnect the battery.");
  Serial.println("Press any key to continue.");
  WaitForKeyStroke();
  ESC.writeMicroseconds(MAX_SIGNAL); // Sending MAX_SIGNAL tells the ESC to enter calibration mode

  Serial.println();
  Serial.println("Calibration step 2. Connect the battery.");
  Serial.println("Wait for two short bips.");
  Serial.println("Press any key to continue.");
  WaitForKeyStroke();

  ESC.writeMicroseconds(MIN_SIGNAL); // Sending MIN_SIGNAL tells the ESC the calibration value
  Serial.println();
  Serial.println("Wait for 4 short bips, and one long bip.");
  Serial.println("Press any key to finish.");
  WaitForKeyStroke();
}

// ================================================================
// Loop
// ================================================================
void loop()
{
  CtrlPWM = map(analogRead(POT_PIN), 0, 4095, 0, 180); // Read the pot, map the reading from [0, 4095] to [0, 180]
  ESC.write(CtrlPWM);                                  // Send the command to the ESC
  SerialDataPrint();                                   // Print data on the serial monitor for debugging
}
// ================================================================
// Function Definition
// ================================================================
void Init_Serial()
{
  Serial.begin(115200);
  while (!Serial)
    ;
}
// ================================================================
void SerialDataPrint()
{
  if (micros() - time_prev >= 20000)
  {
    time_prev = micros();
    Serial.print(millis());
    Serial.print("\t");
    Serial.println(CtrlPWM);
  }
}
// ================================================================
void WaitForKeyStroke()
{
  while (!Serial.available())
    ;
  while (Serial.available())
    Serial.read();
}

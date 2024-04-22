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
void SerialDataPrint(); // Function to print data on the serial monitor
void Init_Serial();     // Function to init the serial monitor
void Init_ESC();        // Function to init the ESC
// ================================================================
// Setup
// ================================================================
void setup()
{
  Init_Serial(); // Initialize the serial communication
  Init_ESC();    // Initialize the ESC
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
void Init_ESC()
{
  ESC.attach(MOTOR_PIN, MIN_SIGNAL, MAX_SIGNAL);
  ESC.writeMicroseconds(MIN_SIGNAL);
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

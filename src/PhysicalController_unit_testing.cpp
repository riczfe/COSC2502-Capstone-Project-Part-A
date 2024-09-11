//////////////////////////////////////// BUTTON ///////////////////////////////////////////////////////////////
// #include <Arduino.h>

// const int buttonPin = 2; // GPIO pin where OUT pin of button is connected
// int buttonState = 0;

// void setup() {
//     Serial.begin(115200); // Initialize serial communication for debugging
//     pinMode(buttonPin, INPUT_PULLDOWN); // Button OUT pin as input (no need for pull-up/pull-down)
// }

// void loop() {
//     // Read the state of the button
//     buttonState = digitalRead(buttonPin);

//     // Check if the button is pressed (buttonState is LOW)
//     if (buttonState == HIGH) {
//         Serial.println("Button pressed!");
//         delay(200); // Debounce delay
//     }else if(buttonState == LOW) {
//         Serial.println("Button not pressed!");
//         delay(200); // Debounce delay
//     }

//     delay(10); // Optional delay to reduce CPU usage
// }

////////////////////////////////////////// JOYSTICK /////////////////////////////////////////////////////////

// #include <Arduino.h>

// // Define analog input pins for X and Y axis of joystick
// const int analogXPin = 25;
// const int analogYPin = 26;


// void setup() {
//   Serial.begin(115200); // Initialize serial communication
 
// }
// int xAxis = 0;
// int yAxis = 0;

// void loop() {
//   // Read analog values from joystick
//   xAxis = analogRead(analogXPin);
//   yAxis = analogRead(analogYPin);


//   // Print values to Serial Monitor
//   Serial.print("X-axis: ");
//   Serial.print(xAxis);
//   Serial.print("\tY-axis: ");
//   Serial.println(yAxis);


// }



//////////////////////////////////////////////////////// KEYPAD /////////////////////////////////////////////////
// #include <Arduino.h>
// #include <Keypad.h>

// const byte ROWS = 1; // Number of rows in the keypad (1 for 4x1 keypad)
// const byte COLS = 4; // Number of columns in the keypad

// // Define the keymap - adjust according to your keypad layout
// char keys[ROWS][COLS] = {
//   {'1', '2', '3', '4'}
// };

// // Define the row and column pin connections to the keypad
// byte rowPins[ROWS] = {4}; // Row R1 connected to GPIO 4
// byte colPins[COLS] = {23, 18, 19, 5}; // Columns C1, C2, C3, C4 connected to GPIO 

// // Create Keypad object
// Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// void setup() {
//   Serial.begin(115200); // Initialize serial communication
// }

// void loop() {
//   char key = keypad.getKey(); // Get key pressed
//   int value = key - '0';
  
//   if (key != NO_KEY) { // Check if a key is pressed
//     Serial.println(key); // Print the key pressed
//   }

//   delay(10); // Adjust delay as needed
// }



///////////////////////////////////////////////// LCD  ///////////////////////////////////////////////////
// #include <Arduino.h>
// #include <Wire.h>
// #include <LiquidCrystal_I2C.h>

// // Set the LCD address to 0x27 for a 16 chars and 2 line display
// LiquidCrystal_I2C lcd(0x27, 16, 2);

// void setup() {
//     // Initialize the LCD
//     Serial.begin(115200);
//     lcd.begin(16,2);
//     lcd.init();
    
//     // Turn on the backlight (optional)
//     lcd.backlight();
    
//     // Print a message to the LCD
//     lcd.setCursor(0, 0); // Set cursor to first column, first row
//     lcd.print("Hello, LCD 1602!");

//     lcd.setCursor(0, 1); // Set cursor to first column, second row
//     lcd.print("ESP32 is awesome!");
// }

// void loop() {
//       // Print a message to the LCD
//     lcd.setCursor(0, 0); // Set cursor to first column, first row
//     lcd.print("Hello, LCD 1602!");

//     lcd.setCursor(0, 1); // Set cursor to first column, second row
//     lcd.print("ESP32 is awesome!");

//     delay(1000);
//     lcd.clear();
//     delay(1000);
// }


///////////////////////////////////////////////////// TIMER, INTERRUPT //////////////////////////////////////////////

// #include <Arduino.h>

// hw_timer_t *timer1 = NULL;
// hw_timer_t *timer2 = NULL;

// const int ledPin1 = 2; // Example LED 1 connected to GPIO 2
// const int ledPin2 = 16; // Example LED 2 connected to GPIO 4
// const int ledPin3 = 4; // Example LED 2 connected to GPIO 4

// bool trigger = true;

// // Define the GPIO pin connected to the pushbutton
// const int buttonPin = 34;
// // Variable to store the state of the button
// volatile bool buttonPressed = false;

// const unsigned long interval1 = 2000; // Interval for LED 1 (2 seconds)
// const unsigned long interval2 = 2000; // Interval for LED 2 (3 seconds)


// volatile bool buttonState = LOW;   // Current state of the button
// volatile bool lastButtonState = LOW;  // Previous state of the button
// volatile unsigned long debounceTime = 0;  // Debounce timer
// const unsigned long debounceDelay = 200;  // Debounce time in milliseconds


// // Interrupt Service Routine (ISR) for the button press
// void IRAM_ATTR buttonISR() {
//     buttonState = digitalRead(buttonPin);
//     if (buttonState != lastButtonState) {
//         // Reset the debounce timer
//         debounceTime = millis();
//     }
//     // Check if enough time has passed to consider it a valid button press
//     if ((millis() - debounceTime) > debounceDelay) {
//         // If the button state has changed and is stable, toggle the LED
//         digitalWrite(ledPin2, trigger);
//         trigger = !trigger;
//         Serial.println("Button pressed");

//         // Update the last button state
//         lastButtonState = buttonState;
//     }

// }

// void IRAM_ATTR onTimer1() {
//     static bool ledState = true;
//     digitalWrite(ledPin1, ledState);
//     ledState = !ledState;
// }

// void IRAM_ATTR onTimer2() {
//     static bool ledState = false;
//     digitalWrite(ledPin3, ledState);
//     ledState = !ledState;
// }

// void setup() {
//     // Initialize serial communication
//     Serial.begin(115200);

//     // Set the button pin as input with internal pull-up resistor enabled
//     pinMode(buttonPin, INPUT_PULLUP);
//     // Attach interrupt to the button pin
//     attachInterrupt(digitalPinToInterrupt(buttonPin), buttonISR, FALLING);

//     // Initialize LED pins
//     pinMode(ledPin1, OUTPUT);
//     pinMode(ledPin2, OUTPUT);
//     pinMode(ledPin3, OUTPUT);

//     // Create timer1 for LED 1
//     timer1 = timerBegin(0, 80, true); // Timer 0, prescaler 80, count up
//     timerAttachInterrupt(timer1, &onTimer1, true); // Attach onTimer1 function
//     timerAlarmWrite(timer1, interval1 * 1000, true); // Set interval1 in microseconds
//     timerAlarmEnable(timer1); // Enable timer1

//     // Create timer2 for LED 2
//     timer2 = timerBegin(1, 80, true); // Timer 1, prescaler 80, count up
//     timerAttachInterrupt(timer2, &onTimer2, true); // Attach onTimer2 function
//     timerAlarmWrite(timer2, interval2 * 1000, true); // Set interval2 in microseconds
//     timerAlarmEnable(timer2); // Enable timer2
// }

// void loop() {
//     Serial.println("whwueu");
//     delay(3000);
// }






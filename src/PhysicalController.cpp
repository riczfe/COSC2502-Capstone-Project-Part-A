// #include <WiFi.h>
// #include <Arduino.h>
// #include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps20.h"
// #include "Wire.h"
// #include <ESP32Servo.h>
// #include <Arduino.h>
// #include <Wire.h>
// #include <LiquidCrystal_I2C.h>
// #include <Keypad.h>
// #include <WiFiServer.h>
// #include <ESPAsyncWebServer.h>
// #include <ArduinoWebsockets.h>
// #include "web.h"


// #define POT_PIN 32    // Potentiometer
// #define BUTTON_LEFT 35
// #define BUTTON_RIGHT 34
// #define BUTTON_SELF_LOCKING 25
// #define MULTI_COLOR_LED 26
// #define LED_1 13
// #define LED_2 12
// #define LED_3 14
// #define LED_4 27
// #define MAX_CLIENTS 4  // Allow maximum 4 drones connected to controller 


// const char* ssid = "RB_MiniDrone";    // SSID of master ESP32's Wi-Fi network
// const char* password = "123456789";     // Password for master ESP32's Wi-Fi network
// const int serverPort = 80;             // Port 80 is default for HTTP
// const char* serverIP = "192.168.4.1"; 
// const char* ssid2 = "Physical_ESP32";    // SSID of master ESP32's Wi-Fi network
// const char* password2 = "123456";     // Password for master ESP32's Wi-Fi network


// MPU6050 mpu;       // Prepare the mpu object to obtain the angles from the DMP
// MPU6050 accelgyro; // Prepare the accelgyro object to obtain the gyroscope and the acceleration data

// // MPU variable
// uint16_t packetSize;    // DMP packet size. Default is 42 bytes.
// uint16_t fifoCount;     // count of all bytes currently in FIFO
// uint8_t fifoBuffer[64]; // FIFO storage buffer
// Quaternion q;           // [w, x, y, z]         quaternion container
// VectorFloat gravity;    // [x, y, z]            gravity vector
// float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll
// int16_t ax, ay, az;     // Raw acceleration data from the MPU
// int16_t gx, gy, gz;     // Raw gyroscope data from the MPU

// float anglex, angley, anglez; // angle in the x, y, z direction
// float gyrox, gyroy, gyroz;    // angle rate in the x, y, z direction
// float accx, accy, accz;       // acceleration in the x, y, z direction

// unsigned long time_prev = 0; // data for the serial communication


// int buttonState_Left = 0, buttonState_Right = 0;
// int buttonState_SL = 0;
// int CtrlPWM = 0;                      
// int Left = 0, Right = 0, Self_lock = 0;
// int current_drone = 1;
// int dataArray[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};     
// int number_of_clients = 0;
// int receivedarrayfromMaster[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // Array to store received values from master
// unsigned long previousMillis = 0;   // Variable to store the previous time
// const long interval = 500;      // Time delay (0.5 sec)

// //------------------------------------------
// // Define server
// WiFiServer server(serverPort);

// //-------------------------------------------

// WiFiClient clients[MAX_CLIENTS + 1]; // Including personal device
// IPAddress clientIPs[MAX_CLIENTS];   // Array to store clients'ip address
// IPAddress clientIP(0, 0, 0, 0);  // Ip address variable with 0.0.0.0

// WiFiClient client;

// // ------------------------------------------LCD----------------------------------------------------------------------------------------------------
// // Set the LCD address to 0x27 for a 16 chars and 2 line display
// LiquidCrystal_I2C lcd(0x27, 16, 2);

// //--------------------------------------------JOYSTICK---------------------------------------------------------------------------------------------------
// // Define analog input pins for X and Y axis of joystick
// const int analogXPin = 36;
// const int analogYPin = 39;

// int xAxis = 0;
// int yAxis = 0;
// //--------------------------------------------KEYPAD---------------------------------------------------------------------------------------------------
// // Define row and col for keypad
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

// char Key_input;
// int Key_value = 0;

// // // ========================================================================================================================================
// // // Function Declaration
// // // ============================================================================================================================================
// void Init_Serial();     // Function to init the serial monitor
// void Init_MPU();        // Function to init the MPU6050
// void Get_MPUangle();    // Function to get the angle from the MPU6050
// void Get_accelgyro();   // Function to get the gyro and acc from the MPU6050
// void Serial_display();
// // void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info);




// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// void setup() {
//     lcd.begin(16,2);
//     lcd.init();
    
//     // Turn on the backlight (optional)
//     lcd.backlight();

//     // Print a message to the LCD
//     lcd.setCursor(0, 0); // Set cursor to first column, first row
//     lcd.print("    RB DRONE    ");
//     lcd.setCursor(0, 1); // Set cursor to first column, second row
//     lcd.print("    WELCOME!    ");
//     delay(2000);
    
//     Serial.begin(115200);
//     pinMode(BUTTON_LEFT, INPUT_PULLDOWN); // Button OUT pin as input (no need for pull-up/pull-down)
//     pinMode(BUTTON_RIGHT, INPUT_PULLDOWN); // Button OUT pin as input (no need for pull-up/pull-down)
//     pinMode(BUTTON_SELF_LOCKING, INPUT_PULLDOWN); // Button OUT pin as input (no need for pull-up/pull-down)
//     pinMode(MULTI_COLOR_LED, OUTPUT);
//     pinMode(13, OUTPUT);
//     pinMode(12, OUTPUT);
//     pinMode(14, OUTPUT);
//     pinMode(27, OUTPUT);


//     Init_MPU();

//     // Welcome screen
//     lcd.clear();
//     lcd.setCursor(0, 0); // Set cursor to first column, first row
//     lcd.print("Control Method:");
//     lcd.setCursor(0, 1); // Set cursor to first column, second row
//     lcd.print("    Physical    ");
//     delay(2000);

//     // Connect to Wi-Fi network
//     WiFi.begin(ssid, password);
//     Serial.print("Connecting to ");
//     Serial.println(ssid);
    
//     // // Wait for Wi-Fi connection
//     // while (WiFi.status() != WL_CONNECTED) {
//     //     delay(1000);
//     //     Serial.print(".");
//     //     lcd.clear();
//     //     lcd.setCursor(0, 0); // Set cursor to first column, first row
//     //     lcd.print(" Connecting...  ");
//     //     lcd.setCursor(0, 1); // Set cursor to first column, second row
//     //     lcd.print("  Please Wait   ");

//     // }
    
//     // Warning after successfully connected
//     Serial.println("");
//     Serial.println("WiFi connected successfully");

//     lcd.clear();
//     lcd.setCursor(0, 0); // Set cursor to first column, first row
//     lcd.print("  Successfully  ");
//     lcd.setCursor(0, 1); // Set cursor to first column, second row
//     lcd.print("--Ready To Use--");
//     delay(2000);

//     server.begin(); 
// }

// // ===================================================================================================================================================
// void Init_MPU()
// {
//     Wire.begin(21, 22);      // Wire.begin(I2C_SDA, I2C_SCL);
//     Wire.setClock(400000);   // Set the SCL clock to 400KHz
//     accelgyro.initialize();  // Initialize the accelgyro
//     mpu.initialize();        // Initialize the MPU
//     mpu.dmpInitialize();     // Initialize the DMP (microchip that calculate the angle on the MPU6050 module)
//     mpu.setDMPEnabled(true); // Enable the DMP
//     packetSize = mpu.dmpGetFIFOPacketSize();
//     mpu.CalibrateAccel(6); // Calibrate the accelerometer
//     mpu.CalibrateGyro(6);  // Calibrate the gyroscope
// }

// // ======================================================================================================================================
// void Get_MPUangle()
// {
//     // Clear buffer
//     mpu.resetFIFO();
//     // Get FIFO count
//     fifoCount = mpu.getFIFOCount();
//     // Wait for the FIFO to be filled with the correct data number
//     while (fifoCount < packetSize)
//         fifoCount = mpu.getFIFOCount();
//     // read a packet from FIFO
//     mpu.getFIFOBytes(fifoBuffer, packetSize);
//     mpu.dmpGetQuaternion(&q, fifoBuffer);
//     mpu.dmpGetGravity(&gravity, &q);
//     mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//     anglex = ypr[2] * 180 / M_PI;
//     angley = -ypr[1] * 180 / M_PI;
//     anglez = -ypr[0] * 180 / M_PI;
// }

// // ==================================================================================================================================================
// void Get_accelgyro()
// {
//     accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//     gyrox = gx / 131.0;
//     gyroy = gy / 131.0;
//     gyroz = gz / 131.0;
//     accx = ax / 16384.;
//     accy = ay / 16384.;
//     accz = az / 16384.;
// }

// // ===========================================================================================================================================
// void Serial_display()
// {
//     Serial.print(Key_value);
//     Serial.print("\t");
//     Serial.print(CtrlPWM);
//     Serial.print("\t");
//     Serial.print(xAxis);
//     Serial.print("\t");
//     Serial.print(yAxis);
//     Serial.print("\t");
//     Serial.print(Left);
//     Serial.print("\t");
//     Serial.print(Right);
//     Serial.print("\t");
//     Serial.print(anglex);
//     Serial.print("\t");
//     Serial.print(angley);
//     Serial.print("\t");
//     Serial.println(anglez);
// }


// int buttonState = 0;
// // ===========================================================================================================================================================
// void loop() {
//     // Ensure client is connected to the server
//     if (!client.connected()) {
//         if (!client.connect(serverIP, serverPort)) {
//         Serial.println("Failed to connect to server. Retrying...");
//         delay(1000);
//         return;
//         }
//         Serial.println("Connected to server");
//     }
    
//     lcd.clear();
//     lcd.setCursor(0, 0); // Set cursor to first column, first row
//     lcd.print("  Setup Stage   ");
//     lcd.setCursor(0, 1); // Set cursor to first column, second row
//     lcd.print("----------------");
//     delay(2000);

//     lcd.clear();
//     lcd.setCursor(0, 0); // Set cursor to first column, first row
//     lcd.print(">Drone Quantity<");
//     lcd.setCursor(0, 1); // Set cursor to first column, second row
//     lcd.print("---Use Keypad---");

//     // REQUEST NUMBER OF DRONE WANT TO CONTROLL ----------------------------------------------------------------------------------------------
//     // Check if no drone number selected
//     while(Key_value == 0){
//         Key_input = keypad.getKey(); // Get key pressed
//         if (Key_input != NO_KEY) { // Check if a key is pressed
//             Key_value = Key_input - '0'; // Print the key pressed
//         }

//         if(Key_value == 1 || Key_value == 2 || Key_value == 3 || Key_value == 4){
//             lcd.clear();
//             lcd.setCursor(0, 0); // Set cursor to first column, first row
//             lcd.print("Drone Controlled");
//             lcd.setCursor(0, 1); // Set cursor to first column, second row
//             lcd.print("    Total: ");
//             lcd.print(Key_value);
//             delay(2000);
//             break;
            
//         }
//     }
    
//     // Connection warning
//     lcd.clear();
//     lcd.setCursor(0, 0); // Set cursor to first column, first row
//     lcd.print("Drone Connection");
//     lcd.setCursor(0, 1); // Set cursor to first column, second row
//     lcd.print("Setting...");
//     delay(2000);

//     //Send number of drones that user what to control to Master drone
//     dataArray[0] = Key_value;
//     client.write((uint8_t*)dataArray, sizeof(dataArray));

//     while(1){
//         // Check for the data send back from Master drone
//         if (client.available()) { 
//             Serial.println("Data read from Master:"); 
//             client.readBytes((uint8_t*)receivedarrayfromMaster, sizeof(receivedarrayfromMaster));

//             for (int i = 0; i < 9; i++) {
//                 Serial.print(receivedarrayfromMaster[i]);
//                 Serial.print("\t");
//             }
//             Serial.println();

//             if(receivedarrayfromMaster[0] == 2){
//                 lcd.clear();
//                 lcd.setCursor(0, 0); // Set cursor to first column, first row
//                 lcd.print("Please connect");
//                 lcd.setCursor(0, 1); // Set cursor to first column, second row
//                 lcd.print("drone number 2");
//                 delay(1000);
//             }else if(receivedarrayfromMaster[0] == 3){
//                 lcd.clear();
//                 lcd.setCursor(0, 0); // Set cursor to first column, first row
//                 lcd.print("Please connect");
//                 lcd.setCursor(0, 1); // Set cursor to first column, second row
//                 lcd.print("drone number 3");
//                 delay(1000);
//             }else if(receivedarrayfromMaster[0] == 4){
//                 lcd.clear();
//                 lcd.setCursor(0, 0); // Set cursor to first column, first row
//                 lcd.print("Please connect");
//                 lcd.setCursor(0, 1); // Set cursor to first column, second row
//                 lcd.print("drone number 4");
//                 delay(1000);
//             }else if(receivedarrayfromMaster[0] == 0){
//                 // Connection successfully warning
//                 lcd.clear();
//                 lcd.setCursor(0, 0); // Set cursor to first column, first row
//                 lcd.print("     Drone     ");
//                 lcd.setCursor(0, 1); // Set cursor to first column, second row
//                 lcd.print("Connection done");
//                 delay(2000);
//                 break;
//             }
//         }      
//     }


//     // // // Clients connected check
//     // // while(1){
//     // //     for(int i=0;i<=MAX_CLIENTS;i++){
//     // //         if(clients[i] && clients[i].connected()){
//     // //             Serial.print(clients[i].remoteIP());
//     // //             Serial.print("\t");
//     // //         }
//     // //     }
//     // //     Serial.println();
//     // // }

//     // // CONTROL SPECIFIC DRONE ----------------------------------------------------------------------------------------------------------------
//     while(1){
//         lcd.clear();
//         lcd.setCursor(0, 0); // Set cursor to first column, first row
//         lcd.print("Ready To Control");
//         lcd.setCursor(0, 1); // Set cursor to first column, second row
//         lcd.print("Data Sending--> ");
//         lcd.print(Key_value);

//         Key_value = 1; //Reset current drone being controlled to Master drone
//         digitalWrite(LED_1, HIGH);
//         digitalWrite(LED_2, LOW);
//         digitalWrite(LED_3, LOW);
//         digitalWrite(LED_4, LOW);
//         int count = 0;
//         while(1){
//             // Get the current time
//             unsigned long currentMillis = millis();  

//             // Get key pressed
//             Key_input = keypad.getKey(); 
//             if (Key_input != NO_KEY) { // Check if a key is pressed
//                 Key_value = Key_input - '0'; // Print the key pressed
//                 if (Key_value == 1 && Key_value != current_drone){
//                     digitalWrite(LED_1, HIGH);
//                     digitalWrite(LED_2, LOW);
//                     digitalWrite(LED_3, LOW);
//                     digitalWrite(LED_4, LOW);
//                     current_drone = 1;
//                 }else if (Key_value == 2 && Key_value != current_drone){
//                     digitalWrite(LED_2, HIGH);
//                     digitalWrite(LED_1, LOW);
//                     digitalWrite(LED_3, LOW);
//                     digitalWrite(LED_4, LOW);
//                     current_drone = 2;
//                 }else if (Key_value == 3 && Key_value != current_drone){
//                     digitalWrite(LED_3, HIGH);
//                     digitalWrite(LED_1, LOW);
//                     digitalWrite(LED_2, LOW);
//                     digitalWrite(LED_4, LOW);
//                     current_drone = 3;
//                 }else if (Key_value == 4 && Key_value != current_drone){
//                     digitalWrite(LED_4, HIGH);
//                     digitalWrite(LED_1, LOW);
//                     digitalWrite(LED_2, LOW);
//                     digitalWrite(LED_3, LOW);
//                     current_drone = 4;
//                 }else if (Key_value == current_drone){
//                     digitalWrite(LED_4, HIGH);
//                     digitalWrite(LED_1, HIGH);
//                     digitalWrite(LED_2, HIGH);
//                     digitalWrite(LED_3, HIGH);
//                     current_drone = 0;
//                     Key_value = 0;
//                 }
//             }
            
//             // Read data from potentiometer
//             CtrlPWM = map(analogRead(POT_PIN), 0, 4095, 0, 255);
                
//             // Read the state of the button
//             // Read left button
//             buttonState_Left = digitalRead(BUTTON_LEFT);
//             // Check if the button is pressed 
//             if (buttonState_Left == HIGH) {
//                 Left = 100;

//             }else{
//                 Left = 0;
//             }
            
//             // Read right button
//             buttonState_Right = digitalRead(BUTTON_RIGHT);
//             // Check if the button is pressed 
//             if (buttonState_Right == HIGH) {
//                 Right = 100;
        
//             }else{
//                 Right = 0;
//             }

//             // Read the state of the self-locking button
//             buttonState_SL = digitalRead(BUTTON_SELF_LOCKING);

//             // Check if the button is pressed (buttonState is LOW)
//             if (buttonState_SL == HIGH) {
//                 Self_lock = 1;
//                 digitalWrite(MULTI_COLOR_LED, HIGH);
//                 // Read data from MPU6050
//                 Get_MPUangle();
//             }else if(buttonState_SL == LOW){
//                 Self_lock = 0;
//                 digitalWrite(MULTI_COLOR_LED, LOW);
//                 anglex = -1;
//                 angley = -1;
//                 anglez = -1;
//             }

//             // Read analog values from joystick
//             xAxis = analogRead(analogXPin);
//             if(xAxis >= 1790){
//                 xAxis = map(analogRead(analogXPin), 1790, 4095, 0, -100);
//             }else if(xAxis <= 1750){
//                 xAxis = map(analogRead(analogXPin), 1750, 0, 0, 100);
//             }else{
//                 xAxis = 0;
//             }
            
//             yAxis = analogRead(analogYPin);
//             if(yAxis >= 1850){
//                 yAxis = map(analogRead(analogYPin), 1790, 4095, 0, -100);
//             }else if(yAxis <= 1810){
//                 yAxis = map(analogRead(analogYPin), 1750, 0, 0, 100);
//             }else{
//                 yAxis = 0;
//             }
            
//             dataArray[0] = Key_value;
//             dataArray[1] = CtrlPWM;
//             dataArray[2] = xAxis;
//             dataArray[3] = yAxis;
//             dataArray[4] = Left;
//             dataArray[5] = Right;
//             dataArray[6] = Self_lock;
//             dataArray[7] = anglex;
//             dataArray[8] = angley;
//             dataArray[9] = anglez;

//             // Send data to Master drone
//             client.write((uint8_t*)dataArray, sizeof(dataArray));

//             Serial_display();

//             if (currentMillis - previousMillis >= interval) {
//             // Save the current time to reset the interval
//             previousMillis = currentMillis;
//             lcd.clear();
//             lcd.setCursor(0, 0); // Set cursor to first column, first row
//             lcd.print("Speed");
//             lcd.setCursor(0, 1); // Set cursor to first column, second row
//             lcd.print(CtrlPWM);
//             }
//         }
//     }  
// }
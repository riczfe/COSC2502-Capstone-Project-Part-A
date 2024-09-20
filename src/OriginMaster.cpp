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
// #define MAX_CLIENTS 4  // Allow maximum 4 drones connected to controller 


// const char* ssid = "TIM-MASTER_ESP32";    // SSID of master ESP32's Wi-Fi network
// const char* password = "password";     // Password for master ESP32's Wi-Fi network

// const char* ssid2 = "Virtual_ESP32";    // SSID of master ESP32's Wi-Fi network
// const char* password2 = "123456";     // Password for master ESP32's Wi-Fi network
// const int serverPort = 80;             // Port 80 is default for HTTP


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
// int CtrlPWM = 0;                      
// int Left = 0, Right = 0;
// int dataArray[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};     
// int dataread[9];
// int number_of_clients = 0;
// int total_drone_being_controlled = 1;   // Start at 1 because 0 is for personal device connected initially
// bool clientConnected[MAX_CLIENTS + 1] = {false};  // Array to track connected clients
// unsigned long previousMillis = 0;   // Variable to store the previous time
// const long interval = 500;      // Time delay (0.5 sec)

// //------------------------------------------
// // Define server
// WiFiServer server(serverPort);

// using namespace websockets;
// WebsocketsServer server2;
// AsyncWebServer webserver(80);

// //-------------------------------------------

// WiFiClient clients[MAX_CLIENTS + 1]; // Including personal device
// IPAddress clientIPs[MAX_CLIENTS];   // Array to store clients'ip address
// IPAddress clientIP(0, 0, 0, 0);  // Ip address variable with 0.0.0.0

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





// int control_method = 0;
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
    
//     lcd.clear();
//     lcd.setCursor(0, 0); // Set cursor to first column, first row
//     lcd.print("Setting...");
//     lcd.setCursor(0, 1); // Set cursor to first column, second row
//     lcd.print("Please wait!");
    
//     Serial.begin(115200);
//     pinMode(BUTTON_LEFT, INPUT_PULLDOWN); // Button OUT pin as input (no need for pull-up/pull-down)
//     pinMode(BUTTON_RIGHT, INPUT_PULLDOWN); // Button OUT pin as input (no need for pull-up/pull-down)

//     Init_MPU();

//     delay(2000);

//     Serial.println("Choose control method");
//     while(1){
//         buttonState_Left = digitalRead(BUTTON_LEFT);
//         if(buttonState_Left == HIGH){
//             control_method = 1;
//             break;
//         }
//         buttonState_Right = digitalRead(BUTTON_RIGHT);
//         if(buttonState_Right == HIGH){
//             control_method = 2;
//             break;
//         }
//     }

//     // Connect to Wi-Fi network
//     WiFi.softAP(ssid, password);
//     Serial.println("\nMaster ESP32 is now running as an access point.");
//     Serial.print("IP Address: ");
//     Serial.println(WiFi.softAPIP());

//     // WiFi.onEvent(WiFiEvent);

//     lcd.clear();
//     lcd.setCursor(0, 0); // Set cursor to first column, first row
//     lcd.print("  Successfully  ");
//     lcd.setCursor(0, 1); // Set cursor to first column, second row
//     lcd.print("IP: 192.168.4.1");
    
//     if(control_method == 1){
//         webserver.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
//         AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", index_html_gz, sizeof(index_html_gz));
//         response->addHeader("Content-Encoding", "gzip");
//         request->send(response);
//         });

//         webserver.begin();
//         server2.listen(82);
//         Serial.print("Is server live? ");
//         Serial.println(server2.available());
//     }else if(control_method ==2){
//        // Start the web server
//         server.begin(); 
//     }
//     // // Initialize clientIPs array
//     // for (int i = 0; i <= MAX_CLIENTS; i++) {
//     //     clientIPs[i] = IPAddress(0, 0, 0, 0);  // Initialize with 0.0.0.0
//     // }
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

// // void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info){
// //     switch(event){
// //         case SYSTEM_EVENT_AP_STACONNECTED:
// //             Serial.println("Connected");
// //             break;
// //         case SYSTEM_EVENT_AP_STADISCONNECTED:
// //             Serial.println("Disconnected");
// //             break;
// //     }
// // }
// // ===========================================================================================================================================================
// int execution=0;
// void loop() {
//     // Start the program after detected any clients had connected to server ip
//     if(server.hasClient()){
//         WiFiClient client = server.available();

//         clients[0] = client;
//         // Detection warining
//         lcd.clear();
//         lcd.setCursor(0, 0); // Set cursor to first column, first row
//         lcd.print("   Connection   ");
//         lcd.setCursor(0, 1); // Set cursor to first column, second row
//         lcd.print("    Detected    ");

//         // WEBSITE WELCOME ---------------------------------------------------------------------------------------------------------------------------------
//         // Read the first line of the request
//         String request = client.readStringUntil('\r');
//         Serial.println(request);
//         client.flush();

//         // Handle control method based on URL
//         if (request.indexOf("/P") != -1) {
//             // Create new blank webpage
//             client.println("HTTP/1.1 200 OK");
//             client.println("Content-Type: text/html");
//             client.println();
//             // Set up for webpage
//             client.println("<html><head><title>Controller Selection</title></head><body>");
//             client.println("<style>");
//             client.println("  body {");
//             client.println("    display: flex;");
//             client.println("    justify-content: center;");
//             client.println("    align-items: center;");
//             client.println("    height: 100vh;");
//             client.println("    margin: 0;");
//             client.println("    font-family: Arial, sans-serif;");
//             client.println("    text-align: center;");
//             client.println("  }");
//             client.println("</style>");
//             client.println("</head><body>");
//             client.println("<h1 style='text-align: center; font-size: 50px; color: blue;'>-Mechanical Controller Method-</p>");
//             client.println("<p style='font-size: 30px; color: black;'>Use controller for further setup</p>");
//             client.println("<p style='font-size: 20px; color: red'>Caution: <span style='color: gray;'>Upcoming settings will not be able to be changed proactively during control stage. In case you want to change, you will have to start over from the beginning.</span></p>");
//             client.println("</body></html>");

//             lcd.clear();
//             lcd.setCursor(0, 0); // Set cursor to first column, first row
//             lcd.print("Control Method:");
//             lcd.setCursor(0, 1); // Set cursor to first column, second row
//             lcd.print("    Physical    ");
//             delay(2000);

//             lcd.clear();
//             lcd.setCursor(0, 0); // Set cursor to first column, first row
//             lcd.print("  Setup Stage   ");
//             lcd.setCursor(0, 1); // Set cursor to first column, second row
//             lcd.print("----------------");
//             delay(2000);

//             lcd.clear();
//             lcd.setCursor(0, 0); // Set cursor to first column, first row
//             lcd.print("Number of drones");
//             lcd.setCursor(0, 1); // Set cursor to first column, second row
//             lcd.print("---Use Keypad---");

//             // REQUEST NUMBER OF DRONE WANT TO CONTROLL ----------------------------------------------------------------------------------------------
//             // Check if no drone number selected
//             while(Key_value == 0){
//                 Key_input = keypad.getKey(); // Get key pressed
//                 if (Key_input != NO_KEY) { // Check if a key is pressed
//                     Key_value = Key_input - '0'; // Print the key pressed
//                 }

//                 if(Key_value == 1 || Key_value == 2 || Key_value == 3 || Key_value == 4){
//                     lcd.clear();
//                     lcd.setCursor(0, 0); // Set cursor to first column, first row
//                     lcd.print("Drone Controlled");
//                     lcd.setCursor(0, 1); // Set cursor to first column, second row
//                     lcd.print("    Total: ");
//                     lcd.print(Key_value);
//                     delay(2000);
//                     break;
                    
//                 }
//             }
            
//             // Connection warning
//             lcd.clear();
//             lcd.setCursor(0, 0); // Set cursor to first column, first row
//             lcd.print("Drone Connection");
//             lcd.setCursor(0, 1); // Set cursor to first column, second row
//             lcd.print("Setting...");
//             delay(2000);

//             while(total_drone_being_controlled <= Key_value){
//                 lcd.clear();
//                 lcd.setCursor(0, 0); // Set cursor to first column, first row
//                 lcd.print("Please connect");
//                 lcd.setCursor(0, 1); // Set cursor to first column, second row
//                 lcd.print("drone number ");
//                 lcd.print(total_drone_being_controlled);
//                 delay(2000);
                
//                 while(clientConnected[total_drone_being_controlled] == false){
//                     clients[total_drone_being_controlled] = server.available();

//                     if(clients[total_drone_being_controlled] && clients[total_drone_being_controlled].connected() && clients[total_drone_being_controlled].remoteIP() != clients[0].remoteIP()){
//                         lcd.clear();
//                         lcd.setCursor(0, 0); // Set cursor to first column, first row
//                         lcd.print("  Drone number  ");
//                         lcd.setCursor(0, 1); // Set cursor to first column, second row
//                         lcd.print("  ");
//                         lcd.print(total_drone_being_controlled);
//                         lcd.print(" connected   ");
//                         clientConnected[total_drone_being_controlled] = true;
//                         total_drone_being_controlled++;
//                         delay(2000);
//                         break;
//                     }
//                 }
//             }

//             // Connection successfully warning
//             lcd.clear();
//             lcd.setCursor(0, 0); // Set cursor to first column, first row
//             lcd.print("     Drone     ");
//             lcd.setCursor(0, 1); // Set cursor to first column, second row
//             lcd.print("Connection done");
//             delay(2000);


//             // // Clients connected check
//             // while(1){
//             //     for(int i=0;i<=MAX_CLIENTS;i++){
//             //         if(clients[i] && clients[i].connected()){
//             //             Serial.print(clients[i].remoteIP());
//             //             Serial.print("\t");
//             //         }
//             //     }
//             //     Serial.println();
//             // }

//             // CONTROL SPECIFIC DRONE ----------------------------------------------------------------------------------------------------------------
//             while(1){
//                 lcd.clear();
//                 lcd.setCursor(0, 0); // Set cursor to first column, first row
//                 lcd.print("Ready To Control");
//                 lcd.setCursor(0, 1); // Set cursor to first column, second row
//                 lcd.print("Data Sending--> ");
//                 lcd.print(Key_value);

//                 int count = 0;
//                 while(1){
//                     // Get the current time
//                     unsigned long currentMillis = millis();  

//                     // Get key pressed
//                     Key_input = keypad.getKey(); 
//                     if (Key_input != NO_KEY) { // Check if a key is pressed
//                         Key_value = Key_input - '0'; // Print the key pressed
//                     }
                    
//                     // Read data from MPU6050
//                     Get_MPUangle();
//                     Get_accelgyro();
                    
//                     // Read data from potentiometer
//                     CtrlPWM = map(analogRead(POT_PIN), 0, 4095, 0, 180);
                        
//                     // Read the state of the button
//                     // Read left button
//                     buttonState_Left = digitalRead(BUTTON_LEFT);
//                     // Check if the button is pressed 
//                     if (buttonState_Left == HIGH) {
//                         Left = 1;

//                     }else{
//                         Left = 0;
//                     }
//                     // Read right button
//                     buttonState_Right = digitalRead(BUTTON_RIGHT);
//                     // Check if the button is pressed 
//                     if (buttonState_Right == HIGH) {
//                         Right = 1;
                
//                     }else{
//                         Right = 0;
//                     }

//                     // Read analog values from joystick
//                     xAxis = analogRead(analogXPin);
//                     yAxis = analogRead(analogYPin);
                    
//                     dataArray[0] = Key_value;
//                     dataArray[1] = CtrlPWM;
//                     dataArray[2] = xAxis;
//                     dataArray[3] = yAxis;
//                     dataArray[4] = Left;
//                     dataArray[5] = Right;
//                     dataArray[6] = anglex;
//                     dataArray[7] = angley;
//                     dataArray[8] = anglez;

//                     // Send data to slave
//                     for(int i=0;i<=MAX_CLIENTS;i++){
//                         if(clients[i] && clients[i].connected()){
//                             clients[i].write((uint8_t*)dataArray, sizeof(dataArray));
//                             // Serial.print("Data sending to ");
//                             // Serial.print(i);
//                             // Serial.print("\t");
//                             Serial_display();
//                             // Serial.println();
//                             // Read data from slave
//                             // if(clients[i].available()){
//                             //     // clients[i].readBytes((uint8_t*)dataread, sizeof(dataread));
//                             //     // for (int i = 0; i <= 8; i++) {
//                             //     //     Serial.print(dataread[i]);
//                             //     //     Serial.print("\t");
//                             //     // }
//                             //     // Serial.println();
//                             //     // Serial.println(count);
//                             //     // count++;
//                             // }
//                         }
//                     }

//                     // Serial_display();

//                     if (currentMillis - previousMillis >= interval) {
//                     // Save the current time to reset the interval
//                     previousMillis = currentMillis;
//                     lcd.clear();
//                     lcd.setCursor(0, 0); // Set cursor to first column, first row
//                     lcd.print("Speed");
//                     lcd.setCursor(0, 1); // Set cursor to first column, second row
//                     lcd.print(CtrlPWM);
//                     }
//                 }
//             }  
//         // =================================== VIRTUAL CONTROLLER ====================================================================================================       
//         }else if (request.indexOf("/V") != -1) {

//                 lcd.clear();
//                 lcd.setCursor(0, 0); // Set cursor to first column, first row
//                 lcd.print("Control Method:");
//                 lcd.setCursor(0, 1); // Set cursor to first column, second row
//                 lcd.print("    Virtual    ");


//         // =================================== DEFAULT WELCOME SCREEN  =============================================================================================
//         }else{
//             // Send a standard HTTP response header
//             client.println("HTTP/1.1 200 OK");
//             client.println("Content-type:text/html");
//             client.println();
            
//             // Send the HTML content
//             client.println("<html><head><title>RB Drone Control</title>");
//             client.println("<style>");
//             client.println("  body {");
//             client.println("    display: flex;");
//             client.println("    justify-content: center;");
//             client.println("    align-items: center;");
//             client.println("    height: 100vh;");
//             client.println("    margin: 0;");
//             client.println("    font-family: Arial, sans-serif;");
//             client.println("    text-align: center;");
//             client.println("  }");
//             client.println("  .container {");
//             client.println("    max-width: 600px;");
//             client.println("    padding: 20px;");
//             client.println("    border: 1px solid #ccc;");
//             client.println("    border-radius: 5px;");
//             client.println("    background-color: #f9f9f9;");
//             client.println("    box-shadow: 0 0 10px rgba(0, 0, 0, 0.1);");
//             client.println("  }");
//             client.println("</style>");
//             client.println("</head><body>");
//             client.println("<div class='container'>");
//             client.println("<h1>Control Methods</h1>");  // Header tag for the title
//             client.println("<p style='font-size: 30px;'>Click <a href=\"/P\">here</a> to control the drone using:</p>");
//             client.println("<p style='font-size: 30px; color: red;'>Mechanical Controller.</p>");
//             client.println("<p style='font-size: 30px;'>Click <a href=\"/V\">here</a> to control the drone using:</p>");
//             client.println("<p style='font-size: 30px; color: red;'>Web Controller.</p>");
//             client.println("</div>");
//             client.println("</body></html>");
//         }
//     }
// }
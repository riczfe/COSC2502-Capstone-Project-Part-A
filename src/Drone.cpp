// #include <WiFi.h>
// #include <Arduino.h>
// #include "MPU6050_6Axis_MotionApps20.h"
// #include "Wire.h"
// #include <ESP32Servo.h>
// #include <Wire.h>
// #include <WiFiServer.h>
// #include <ESPAsyncWebServer.h>
// #include <ArduinoWebsockets.h>
// #include "web.h"
// #include "MyPID.h"


// #define CURRENT_LED 32    // Potentiometer
// #define BUTTON_RED 4       //34
// #define BUTTON_YELLOW 2        //35
// #define VIRTUAL_LED 16
// #define PHYSICAL_LED 17
// #define MAX_CLIENTS 4  // Allow maximum 4 drones connected to controller 

// #define MOT_1 26
// #define MOT_2 33
// #define MOT_3 27
// #define MOT_4 25


// const char* ssid = "RB_MiniDrone";    // SSID of master ESP32's Wi-Fi network
// const char* password = "123456789";     // Password for master ESP32's Wi-Fi network
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

// double anglex, angley, anglez; // angle in the x, y, z direction
// float gyrox, gyroy, gyroz;    // angle rate in the x, y, z direction
// float accx, accy, accz;       // acceleration in the x, y, z direction

// unsigned long time_prev = 0; // data for the serial communication

// int control_method = 0;
// int buttonState_Left = 0, buttonState_Right = 0;
// int CtrlPWM = 0;                      
// int Left = 0, Right = 0;
// int dataArray[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};     
// int dataread[10];
// int number_of_clients = 0;
// int total_drone_being_controlled = 2;   // Start at 2 because 1 is for Master drone already
// bool clientConnected[MAX_CLIENTS + 1] = {false};  // Array to track connected clients
// unsigned long previousMillis = 0;   // Variable to store the previous time
// const long interval = 500;      // Time delay (0.5 sec)
// int receivedarray[10];  // Array to store received values from master
// int connection_check_array[10];
// int drone_1_speed = 0;

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


// // // ========================================================================================================================================
// // // Function Declaration
// // // ============================================================================================================================================
// void Init_Serial();     // Function to init the serial monitor
// void Init_MPU();        // Function to init the MPU6050
// void Get_MPUangle();    // Function to get the angle from the MPU6050
// void Get_accelgyro();   // Function to get the gyro and acc from the MPU6050
// void Serial_display();


// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// void setup() {
//     Serial.begin(115200);
//     pinMode(BUTTON_YELLOW, INPUT_PULLDOWN); // Button OUT pin as input (no need for pull-up/pull-down)
//     pinMode(BUTTON_RED, INPUT_PULLDOWN); // Button OUT pin as input (no need for pull-up/pull-down)
//     pinMode(PHYSICAL_LED, OUTPUT);
//     pinMode(VIRTUAL_LED, OUTPUT);
//     pinMode(CURRENT_LED, OUTPUT);

//     pinMode(MOT_1, OUTPUT);
//     pinMode(MOT_2, OUTPUT);
//     pinMode(MOT_3, OUTPUT);
//     pinMode(MOT_4, OUTPUT);


//     Init_MPU();
//     Init_PID();      // Initialize the PID

//     delay(1000);

//     Serial.println("\nChoose control method");

//     while(1){ 
//         //-----------------Virtual Method----------------------------------------
//         buttonState_Left = digitalRead(BUTTON_RED);
//         if(buttonState_Left == HIGH){
//             control_method = 1;
//             break;
//         }
//         //-----------------Physical Method-----------------------------------------
//         buttonState_Right = digitalRead(BUTTON_YELLOW);
//         if(buttonState_Right == HIGH){
//             control_method = 2;
//             break;
//         }
//     }

//     // Emit Wi-Fi network
//     WiFi.softAP(ssid, password);
//     Serial.println("\nMaster ESP32 is now running as an access point.");
//     Serial.print("IP Address: ");
//     Serial.println(WiFi.softAPIP());
    
//     //-------------------------------------------------Red button----------------------------------------
//     if(control_method == 1){        
//         webserver.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
//         AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", index_html_gz, sizeof(index_html_gz));
//         response->addHeader("Content-Encoding", "gzip");
//         request->send(response);
//         });
//         Serial.println("Virtual control");
//         digitalWrite(VIRTUAL_LED, HIGH);
//         webserver.begin();
//         server2.listen(82);
//     }
//     //--------------------------------------------------Yellow button--------------------------------------------
//     else if(control_method ==2){
//         // Start the web server
//         digitalWrite(PHYSICAL_LED, HIGH);
//         Serial.println("Physical control");
//         server.begin(); 
//     }
// }



// // ===========================================================================================================================================
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
//     Serial.print(anglex);
//     Serial.print("\t");
//     Serial.print(angley);
//     Serial.print("\t");
//     Serial.println(anglez);
// }

// void loop() {
//     while(control_method == 1){  // Virtual_controller

//     }

//     while(control_method == 2){  // Physical_controller
//         // digitalWrite(MOT_1, HIGH);
//         // digitalWrite(MOT_2, HIGH);
//         // digitalWrite(MOT_3, HIGH);
//         // digitalWrite(MOT_4, HIGH);

//         // Detect for controller connection
//         if(server.hasClient()){
//             Serial.println("server has client");
//             WiFiClient client = server.available();
//             clients[0] = client;  // This client will be the controller 

//             // Detection warining
//             Serial.println("Controller connected");

//             // // Create new blank webpage
//             // client.println("HTTP/1.1 200 OK");
//             // client.println("Content-Type: text/html");
//             // client.println();
//             // // Set up for webpage
//             // client.println("<html><head><title>Controller Selection</title></head><body>");
//             // client.println("<style>");
//             // client.println("  body {");
//             // client.println("    display: flex;");
//             // client.println("    justify-content: center;");
//             // client.println("    align-items: center;");
//             // client.println("    height: 100vh;");
//             // client.println("    margin: 0;");
//             // client.println("    font-family: Arial, sans-serif;");
//             // client.println("    text-align: center;");
//             // client.println("  }");
//             // client.println("</style>");
//             // client.println("</head><body>");
//             // client.println("<h1 style='text-align: center; font-size: 50px; color: blue;'>-Mechanical Controller Method-</p>");
//             // client.println("<p style='font-size: 30px; color: black;'>Use controller for further setup</p>");
//             // client.println("<p style='font-size: 20px; color: red'>Caution: <span style='color: gray;'>Upcoming settings will not be able to be changed proactively during control stage. In case you want to change control method, you will have to start over from the beginning.</span></p>");
//             // client.println("</body></html>");
           

//             while(1){
//                 // Check number of drone input from controller 
//                 if (clients[0].available()) {  
//                     clients[0].readBytes((uint8_t*)receivedarray, sizeof(receivedarray));

//                     // Print received data
//                     for (int i = 0; i < 9; i++) {
//                         Serial.print(receivedarray[i]);
//                         Serial.print("\t");
//                     }
//                     Serial.println();

//                     // Notice controller that the setting had completed if request is 1 drone
//                     if (receivedarray[0] == 1){
//                         // Send data back to controller
//                         clients[0].write((uint8_t*)receivedarray, sizeof(receivedarray));
//                     }
//                     break;
//                 } 
//             }

//             while(total_drone_being_controlled <= receivedarray[0]){
//                 for(int i = receivedarray[0]-1; i>=1; i--){
//                     Serial.print("Please connect drone number ");
//                     Serial.println(total_drone_being_controlled);

//                     connection_check_array[0] = total_drone_being_controlled;
//                     clients[0].write((uint8_t*)connection_check_array, sizeof(connection_check_array));

//                     while(clientConnected[total_drone_being_controlled] == false){
//                         clients[total_drone_being_controlled-1] = server.available();

//                         if(clients[total_drone_being_controlled-1] && clients[total_drone_being_controlled-1].connected() && clients[total_drone_being_controlled-1].remoteIP() != clients[total_drone_being_controlled-2].remoteIP()){
//                             Serial.print("Drone number ");
//                             Serial.print(total_drone_being_controlled);
//                              Serial.println(" connected");
//                             clientConnected[total_drone_being_controlled] = true;
//                             total_drone_being_controlled++;
//                             break;
//                         }
//                     }
//                 }
//             }

//             Serial.println("Setting successfully");
//             connection_check_array[0] = 0;
//             clients[0].write((uint8_t*)connection_check_array, sizeof(connection_check_array));
            
            
//             while(1){
//                 if (clients[0].available()){
//                     clients[0].readBytes((uint8_t*)receivedarray, sizeof(receivedarray));

//                     // Send data to first drone slave
//                     clients[1].write((uint8_t*)receivedarray, sizeof(receivedarray));


//                     // Print received data
//                     for (int i = 0; i < 10; i++) {
//                         Serial.print(receivedarray[i]);
//                         Serial.print("\t");
//                     }
                    

//                     // Components control mode
//                     if(receivedarray[7] == -1000 && receivedarray[8] == -1000 && receivedarray[9] == -1000){
//                         // Turn on/off led to indicates current drone being control
//                         if(receivedarray[0] == 1 || receivedarray[0] == 0){
//                             digitalWrite(CURRENT_LED, HIGH);

//                             //Speed adjust according to control signal without tunning
//                             if(receivedarray[2] > 5 || receivedarray[3] > 5 || receivedarray[4] > 5 || receivedarray[5] > 5 || receivedarray[2] < -5 || receivedarray[3] < -5 || receivedarray[4] < -5 || receivedarray[5] < -5){
//                                 Serial.println();
//                                 // Send data from potentiometer to motor
//                                 analogWrite(MOT_1, receivedarray[1] - receivedarray[2] + receivedarray[3] + receivedarray[4] - receivedarray[5]);
//                                 analogWrite(MOT_2, receivedarray[1] - receivedarray[2] - receivedarray[3] - receivedarray[4] + receivedarray[5]);
//                                 analogWrite(MOT_3, receivedarray[1] + receivedarray[2] - receivedarray[3] + receivedarray[4] - receivedarray[5]);
//                                 analogWrite(MOT_4, receivedarray[1] + receivedarray[2] + receivedarray[3] - receivedarray[4] + receivedarray[5]);

//                                 drone_1_speed = receivedarray[1];  // store current speed value in case user changed drone

//                             // Start tunning when stop adjust speed and having specific speed
//                             }else if(receivedarray[2] < 10 && receivedarray[3] < 10 && receivedarray[4] < 10 && receivedarray[5] < 10 && receivedarray[1] >=30){ 
//                                 // Read data from MPU6050 on drone and compute pid
//                                 Get_MPUangle();
//                                 Get_accelgyro();
//                                 Compute_PID();

//                                 // Fiz for x y z
//                                 analogWrite(MOT_1, receivedarray[1] + motor_cmd_x - motor_cmd_y - motor_cmd_z);
//                                 analogWrite(MOT_2, receivedarray[1] - motor_cmd_x - motor_cmd_y + motor_cmd_z);
//                                 analogWrite(MOT_3, receivedarray[1] - motor_cmd_x + motor_cmd_y - motor_cmd_z);
//                                 analogWrite(MOT_4, receivedarray[1] + motor_cmd_x + motor_cmd_y + motor_cmd_z);

//                                 drone_1_speed = receivedarray[1];  // store current speed value in case user changed drone
                                
//                             // Begining run     
//                             }else{
//                                 Serial.println();
//                                 analogWrite(MOT_1, receivedarray[1]);
//                                 analogWrite(MOT_2, receivedarray[1]);
//                                 analogWrite(MOT_3, receivedarray[1]);
//                                 analogWrite(MOT_4, receivedarray[1]);
//                             }

//                         }else if(receivedarray[0] != 1 && receivedarray[0] != 0){
//                             digitalWrite(CURRENT_LED, LOW);
//                             analogWrite(MOT_1, drone_1_speed);
//                             analogWrite(MOT_2, drone_1_speed);
//                             analogWrite(MOT_3, drone_1_speed);
//                             analogWrite(MOT_4, drone_1_speed);
        
//                         }

//                     // IMU control mode 
//                     }else if(receivedarray[7] != -1000 && receivedarray[8] != -1000 && receivedarray[9] != -1000){
//                         if(receivedarray[0] == 1 || receivedarray[0] == 0){
                
//                         }else if(receivedarray[0] != 1 && receivedarray[0] != 0){

//                         }
//                     }
//                 }
//             }
//         }
//     } 
// }




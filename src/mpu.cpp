// #include <Arduino.h>
// #include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps20.h"
// #include "Wire.h"
// #include <ESP32Servo.h>

// // ================================================================
// // Variable declaration
// // ================================================================
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
// // Potentiomer
// #define MAX_SIGNAL 2000 // Parameter required for the ESC definition
// #define MIN_SIGNAL 1000 // Parameter required for the ESC definition
// #define MOTOR_PIN 13    // Pin 13 attached to the ESC signal pin
// #define POT_PIN 4       // Pin 4 attached to the potentiometer

// Servo ESC;                   // Define the ESC
// int CtrlPWM;                 // Control Signal. Varies between [0 - 180]


// // ================================================================
// // Function Declaration
// // ================================================================
// void Init_Serial();     // Function to init the serial monitor
// void Init_MPU();        // Function to init the MPU6050
// void Get_MPUangle();    // Function to get the angle from the MPU6050
// void Get_accelgyro();   // Function to get the gyro and acc from the MPU6050
// void SerialDataPrint(); // Function to print data on the serial monitor
// void SerialDataPrintPotentiometer(); // Function to print data on the serial monitor
// void Init_ESC();        // Function to init the ESC

// // ================================================================
// // Setup function
// // ================================================================
// void setup()
// {
//   Init_Serial();
//   Init_MPU();
// }

// // ================================================================
// // Loop function
// // ================================================================
// void loop()
// {
//   CtrlPWM = map(analogRead(POT_PIN), 0, 4095, 0, 180); // Read the pot, map the reading from [0, 4095] to [0, 180]
//   ESC.write(CtrlPWM);                                  // Send the command to the ESC 
//   Get_MPUangle();
//   Get_accelgyro();
//   SerialDataPrint();

// }

// // ================================================================
// // Function Definition
// // ================================================================
// void Init_Serial()
// {
//   Serial.begin(115200);
//   while (!Serial)
//     ;
// }
// // ================================================================
// void Init_MPU()
// {
//   Wire.begin(21, 22);      // Wire.begin(I2C_SDA, I2C_SCL);
//   Wire.setClock(400000);   // Set the SCL clock to 400KHz
//   accelgyro.initialize();  // Initialize the accelgyro
//   mpu.initialize();        // Initialize the MPU
//   mpu.dmpInitialize();     // Initialize the DMP (microchip that calculate the angle on the MPU6050 module)
//   mpu.setDMPEnabled(true); // Enable the DMP
//   packetSize = mpu.dmpGetFIFOPacketSize();
//   mpu.CalibrateAccel(6); // Calibrate the accelerometer
//   mpu.CalibrateGyro(6);  // Calibrate the gyroscope
// }
// // ================================================================
// void Init_ESC()
// {
//   ESC.attach(MOTOR_PIN, MIN_SIGNAL, MAX_SIGNAL);
//   ESC.writeMicroseconds(MIN_SIGNAL);
// }
// // ================================================================
// void Get_MPUangle()
// {
//   // Clear buffer
//   mpu.resetFIFO();
//   // Get FIFO count
//   fifoCount = mpu.getFIFOCount();
//   // Wait for the FIFO to be filled with the correct data number
//   while (fifoCount < packetSize)
//     fifoCount = mpu.getFIFOCount();
//   // read a packet from FIFO
//   mpu.getFIFOBytes(fifoBuffer, packetSize);
//   mpu.dmpGetQuaternion(&q, fifoBuffer);
//   mpu.dmpGetGravity(&gravity, &q);
//   mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//   anglex = ypr[2] * 180 / M_PI;
//   angley = -ypr[1] * 180 / M_PI;
//   anglez = -ypr[0] * 180 / M_PI;
// }
// // ================================================================
// void Get_accelgyro()
// {
//   accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//   gyrox = gx / 131.0;
//   gyroy = gy / 131.0;
//   gyroz = gz / 131.0;
//   accx = ax / 16384.;
//   accy = ay / 16384.;
//   accz = az / 16384.;
// }
// // ================================================================
// void SerialDataPrint()
// {
//   if (micros() - time_prev >= 1000){
//     time_prev = micros();
//     Serial.print(millis());
//     Serial.print("\t");
//     Serial.print(anglex);
//     Serial.print("\t");
//     Serial.print(angley);
//     Serial.print("\t");
//     Serial.print(anglez);
//     Serial.print("\t");
//     Serial.print(gyrox);
//     Serial.print("\t");
//     Serial.print(gyroy);
//     Serial.print("\t");
//     Serial.print(gyroz);
//     Serial.println();
//     delay(200);
//     // Serial.print("\t");

//     // Serial.print(anglex); //desired yaw pitch roll
//     // Serial.print("\t");
//     // Serial.print(angley);
//     // Serial.print("\t");
//     // Serial.print(anglez);
//     // Serial.print("\t");
//     // Serial.print(CtrlPWM);
//     // Serial.print("\t");
//     // Serial.print(CtrlPWM);
//     // Serial.print("\t");
//     // Serial.print(CtrlPWM);
//     // Serial.print("\t");
//     // Serial.print(CtrlPWM);
//     Serial.print("\n");
    
//   }
// }





// //         lcd.clear();
// //         lcd.setCursor(0, 0); // Set cursor to first column, first row
// //         lcd.print("   Connection   ");
// //         lcd.setCursor(0, 1); // Set cursor to first column, second row
// //         lcd.print("    Detected    ");

// //         // Read the first line of the request
// //         String request = client.readStringUntil('\r');
// //         Serial.println(request);
// //         client.flush();

// //         // Send a standard HTTP response header
// //         client.println("HTTP/1.1 200 OK");
// //         client.println("Content-type:text/html");
// //         client.println();
        
// //         // Send the HTML content
// //         client.println("<html><head><title>RB Drone Control</title>");
// //         client.println("<style>");
// //         client.println("  body {");
// //         client.println("    display: flex;");
// //         client.println("    justify-content: center;");
// //         client.println("    align-items: center;");
// //         client.println("    height: 100vh;");
// //         client.println("    margin: 0;");
// //         client.println("    font-family: Arial, sans-serif;");
// //         client.println("    text-align: center;");
// //         client.println("  }");
// //         client.println("  .container {");
// //         client.println("    max-width: 600px;");
// //         client.println("    padding: 20px;");
// //         client.println("    border: 1px solid #ccc;");
// //         client.println("    border-radius: 5px;");
// //         client.println("    background-color: #f9f9f9;");
// //         client.println("    box-shadow: 0 0 10px rgba(0, 0, 0, 0.1);");
// //         client.println("  }");
// //         client.println("</style>");
// //         client.println("</head><body>");
// //         client.println("<div class='container'>");
// //         client.println("<h1>Control Methods</h1>");  // Header tag for the title
// //         client.println("<p style='font-size: 30px;'>Click <a href=\"/P\">here</a> to control the drone using:</p>");
// //         client.println("<p style='font-size: 30px; color: red;'>Mechanical Controller.</p>");
// //         client.println("<p style='font-size: 30px;'>Click <a href=\"/V\">here</a> to control the drone using:</p>");
// //         client.println("<p style='font-size: 30px; color: red;'>Web Controller.</p>");
// //         client.println("</div>");
// //         client.println("</body></html>");

// //         // Handle control method based on URL
// //         if (request.indexOf("/P") != -1) {
// //             client.println("HTTP/1.1 200 OK");
// //             client.println("Content-Type: text/html");
// //             client.println();
// //             client.println("<html><head><title>Controller Selection</title></head><body>");
// //             client.println("<h1>You choose mechanical controller</h1>");
// //             client.println("</body></html>");
// //             lcd.clear();
// //             lcd.setCursor(0, 0); // Set cursor to first column, first row
// //             lcd.print("Control Method:");
// //             lcd.setCursor(0, 1); // Set cursor to first column, second row
// //             lcd.print("    Physical    ");
// //             delay(2000);

// //             lcd.clear();
// //             lcd.setCursor(0, 0); // Set cursor to first column, first row
// //             lcd.print("Choose drone NO:");
// //             lcd.setCursor(0, 1); // Set cursor to first column, second row
// //             lcd.print("   Use Keypad   ");

// //             // Check if no drone number selected
// //             while(Key_value == 0){
// //                 Key_input = keypad.getKey(); // Get key pressed
// //                 if (Key_input != NO_KEY) { // Check if a key is pressed
// //                     Key_value = Key_input - '0'; // Print the key pressed
// //                 }
// //             }
            
// //             while(1){
// //                 lcd.clear();
// //                 lcd.setCursor(0, 0); // Set cursor to first column, first row
// //                 lcd.print(" Control Drone:");
// //                 lcd.setCursor(0, 1); // Set cursor to first column, second row
// //                 lcd.print("   Number: 0");
// //                 lcd.print(Key_value);
// //                 delay(2000);

// //                 lcd.clear();
// //                 lcd.setCursor(0, 0); // Set cursor to first column, first row
// //                 lcd.print("Ready To Control");
// //                 lcd.setCursor(0, 1); // Set cursor to first column, second row
// //                 lcd.print("Data Sending: 0");
// //                 lcd.print(Key_value);

// //                 int current_Key_value = Key_value;

// //                 while(Key_value == current_Key_value){
// //                     // Get key pressed
// //                     Key_input = keypad.getKey(); 
// //                     if (Key_input != NO_KEY) { // Check if a key is pressed
// //                         Key_value = Key_input - '0'; // Print the key pressed
// //                     }
                    
// //                     // Read data from MPU6050
// //                     Get_MPUangle();
// //                     Get_accelgyro();
                    
// //                     // Read data from potentiometer
// //                     CtrlPWM = map(analogRead(POT_PIN), 0, 4095, 0, 180);
                        
// //                     // Read the state of the button
// //                     // Read left button
// //                     buttonState_Left = digitalRead(BUTTON_LEFT);
// //                     // Check if the button is pressed 
// //                     if (buttonState_Left == HIGH) {
// //                         Left = 1;

// //                     }else{
// //                         Left = 0;
// //                     }
// //                     // Read right button
// //                     buttonState_Right = digitalRead(BUTTON_RIGHT);
// //                     // Check if the button is pressed 
// //                     if (buttonState_Right == HIGH) {
// //                         Right = 1;
                
// //                     }else{
// //                         Right = 0;
// //                     }

// //                     // Read analog values from joystick
// //                     xAxis = analogRead(analogXPin);
// //                     yAxis = analogRead(analogYPin);

// //                     // Display data on serial monitor
// //                     Serial_display();
                    
// //                     dataArray[0] = Key_value;
// //                     dataArray[1] = CtrlPWM;
// //                     dataArray[2] = xAxis;
// //                     dataArray[3] = yAxis;
// //                     dataArray[4] = Left;
// //                     dataArray[5] = Right;
// //                     dataArray[6] = anglex;
// //                     dataArray[7] = angley;
// //                     dataArray[8] = anglez;

// //                     // Send data to slave
// //                     client.write((uint8_t*)dataArray, sizeof(dataArray));

                    
// //                     // // Read data from slave
// //                     // client.readBytes((uint8_t*)dataread, sizeof(dataread));
// //                     // Serial.print("Received data from Slave:   ");
// //                     // for (int i = 0; i <= 8; i++) {
// //                     //     Serial.print(dataread[i]);
// //                     //     Serial.print("\t");
// //                     // }
// //                 }
// //             }         
// //         }else if (request.indexOf("/V") != -1) {

// //                 lcd.clear();
// //                 lcd.setCursor(0, 0); // Set cursor to first column, first row
// //                 lcd.print("Control Method:");
// //                 lcd.setCursor(0, 1); // Set cursor to first column, second row
// //                 lcd.print("    Virtual    ");
                
// //                 while(1){}
// //         }
















// //         data send to slave 
// //          WiFiClient client = server.available();

// // if (client && client.connected()) {
// //     Serial.println(client.remoteIP());
// //     for(int i=0;i<=8;i++){
// //         dataArray[i] = i;
// //     }
// //     Serial.println("Data sending");
// //     client.write((uint8_t*)dataArray, sizeof(dataArray));


// // }




// // 2 clients connected
// // numClients = WiFi.softAPgetStationNum();  // Check for number of clients being connected 
// // while(clientConnected[0] == false){
//     //     clients[0] = server.available();
//     //     if(clients[0] && clients[0].connected()){
//     //         clients[0].setTimeout(50);
//     //         Serial.println("Slot 0 having client");
//     //         clientConnected[0] = true;
//     //     }
//     // }

//     // lcd.clear();
//     // lcd.setCursor(0, 0); // Set cursor to first column, first row
//     // lcd.print("   0   ");
//     // lcd.setCursor(0, 1); // Set cursor to first column, second row
//     // lcd.print("    connected    ");

//     // while(clientConnected[1] == false){
//     //     clients[1] = server.available();

//     //     if(clients[1] && clients[1].connected() && clients[1].remoteIP() != clients[0].remoteIP()){
//     //         clients[1].setTimeout(50);
//     //         Serial.println("Slot 0 having client");
//     //         clientConnected[1] = true;
//     //     }
//     // }

//     // lcd.clear();
//     // lcd.setCursor(0, 0); // Set cursor to first column, first row
//     // lcd.print("   1   ");
//     // lcd.setCursor(0, 1); // Set cursor to first column, second row
//     // lcd.print("    connected    ");

//     // if(clients[0]&&clients[0].connected()){
//     //     clients[0].setTimeout(50);
//     //     Serial.print("Data sending to ");
//     //     Serial.println(clients[0].remoteIP());
//     //     Serial.print("Data sending to ");
//     //     clients[0].write((uint8_t*)dataArray, sizeof(dataArray));
//     // }

    
//     // if(clients[1]&&clients[1].connected()){
//     //     clients[1].setTimeout(50);
//     //     Serial.print("Data sending to ");
//     //     Serial.println(clients[1].remoteIP());
//     //     clients[1].write((uint8_t*)dataArray, sizeof(dataArray));
//     // }

















   
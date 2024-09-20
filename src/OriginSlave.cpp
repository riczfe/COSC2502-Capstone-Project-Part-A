// #include <WiFi.h>
// #include <Arduino.h>
// #include "Wire.h"
// #include <ESP32Servo.h>
// #include <Arduino.h>
// #include <Wire.h>
// #include <WiFiServer.h>
// #include "MyPID.h"
// #include "MyMPU.h"

// #define CURRENT_LED 32    
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
// const char* password = "123456789";         // Password for master ESP32's Wi-Fi network
// const char* serverIP = "192.168.4.1";      // IP address of master ESP32
// const int serverPort = 80;                 // Port number for server 

// int receivedarray[10];  // Array to store received values from master
// int slavesendarray[] = {1,2,3,4,5,6,7,8,9};
// int drone_1_speed = 0;

// WiFiClient client;


// void setup() {
//     Serial.begin(115200);
    
//     // Connect to Wi-Fi network
//     WiFi.begin(ssid, password);
//     Serial.print("Connecting to ");
//     Serial.println(ssid);
    
//     // Wait for Wi-Fi connection
//     while (WiFi.status() != WL_CONNECTED) {
//         delay(1000);
//         Serial.print(".");
//     }

//     pinMode(CURRENT_LED, OUTPUT);
    
//     // Warning after successfully connected
//     Serial.println("");
//     Serial.println("WiFi connected successfully");

// }

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

//     // // Check and read data from master ESP32
//     // if (client.available()) {
//     //     client.readBytes((uint8_t*)receivedarray, sizeof(receivedarray));

//     //     // Print received data
//     //     for (int i = 0; i < 9; i++) {
//     //     Serial.print(receivedarray[i]);
//     //     Serial.print("\t");
//     //     }
//     //     Serial.println();

//     //     // // Send data back to Master
//     //     // client.write((uint8_t*)slavesendarray, sizeof(slavesendarray));
//     // }  

//     if (client.available()){
//         client.readBytes((uint8_t*)receivedarray, sizeof(receivedarray));

//         // Print received data
//         for (int i = 0; i < 10; i++) {
//             Serial.print(receivedarray[i]);
//             Serial.print("\t");
//         }
//         Serial.println();
        
//         if(receivedarray[0] == 2 || receivedarray[0] == 0){
//             digitalWrite(CURRENT_LED, HIGH);
//         }else{
//             digitalWrite(CURRENT_LED, LOW);
//         }

//         // // Components control mode
//         // if(receivedarray[7] == -1000 && receivedarray[8] == -1000 && receivedarray[9] == -1000){
//         //     // Turn on/off led to indicates current drone being control
//         //     if(receivedarray[0] == 2 || receivedarray[0] == 0){
//         //         digitalWrite(CURRENT_LED, HIGH);

//         //         //Speed adjust according to control signal without tunning
//         //         if(receivedarray[2] > 5 || receivedarray[3] > 5 || receivedarray[4] > 5 || receivedarray[5] > 5 || receivedarray[2] < -5 || receivedarray[3] < -5 || receivedarray[4] < -5 || receivedarray[5] < -5){
//         //             Serial.println();
//         //             // Send data from potentiometer to motor
//         //             analogWrite(MOT_1, receivedarray[1] - receivedarray[2] + receivedarray[3] + receivedarray[4] - receivedarray[5]);
//         //             analogWrite(MOT_2, receivedarray[1] - receivedarray[2] - receivedarray[3] - receivedarray[4] + receivedarray[5]);
//         //             analogWrite(MOT_3, receivedarray[1] + receivedarray[2] - receivedarray[3] + receivedarray[4] - receivedarray[5]);
//         //             analogWrite(MOT_4, receivedarray[1] + receivedarray[2] + receivedarray[3] - receivedarray[4] + receivedarray[5]);

//         //             drone_1_speed = receivedarray[1];  // store current speed value in case user changed drone

//         //         // Start tunning when stop adjust speed and having specific speed
//         //         }else if(receivedarray[2] < 10 && receivedarray[3] < 10 && receivedarray[4] < 10 && receivedarray[5] < 10 && receivedarray[1] >=30){ 
//         //             // Read data from MPU6050 on drone and compute pid
//         //             Get_MPUangle();
//         //             Get_accelgyro();
//         //             Compute_PID();

//         //             Serial.print(motor_cmd_x);
//         //             Serial.print("\t");
//         //             Serial.print(motor_cmd_y);
//         //             Serial.print("\t");
//         //             Serial.print(motor_cmd_z);
//         //             Serial.println();

//         //             // // Fix for x
//         //             // analogWrite(MOT_1, receivedarray[1] + motor_cmd_x);
//         //             // analogWrite(MOT_2, receivedarray[1] - motor_cmd_x);
//         //             // analogWrite(MOT_3, receivedarray[1] - motor_cmd_x);
//         //             // analogWrite(MOT_4, receivedarray[1] + motor_cmd_x);

//         //             // // Fix for y
//         //             // analogWrite(MOT_1, receivedarray[1] - motor_cmd_y);
//         //             // analogWrite(MOT_2, receivedarray[1] - motor_cmd_y);
//         //             // analogWrite(MOT_3, receivedarray[1] + motor_cmd_y);
//         //             // analogWrite(MOT_4, receivedarray[1] + motor_cmd_y);

//         //             // // Fix for z
//         //             // analogWrite(MOT_1, receivedarray[1] - motor_cmd_z);
//         //             // analogWrite(MOT_2, receivedarray[1] + motor_cmd_z);
//         //             // analogWrite(MOT_3, receivedarray[1] - motor_cmd_z);
//         //             // analogWrite(MOT_4, receivedarray[1] + motor_cmd_z);

//         //             // Fiz for x y z
//         //             analogWrite(MOT_1, receivedarray[1] + motor_cmd_x - motor_cmd_y - motor_cmd_z);
//         //             analogWrite(MOT_2, receivedarray[1] - motor_cmd_x - motor_cmd_y + motor_cmd_z);
//         //             analogWrite(MOT_3, receivedarray[1] - motor_cmd_x + motor_cmd_y - motor_cmd_z);
//         //             analogWrite(MOT_4, receivedarray[1] + motor_cmd_x + motor_cmd_y + motor_cmd_z);

//         //             drone_1_speed = receivedarray[1];  // store current speed value in case user changed drone
                    
//         //         // Begining run     
//         //         }else{
//         //             Serial.println();
//         //             analogWrite(MOT_1, receivedarray[1]);
//         //             analogWrite(MOT_2, receivedarray[1]);
//         //             analogWrite(MOT_3, receivedarray[1]);
//         //             analogWrite(MOT_4, receivedarray[1]);
//         //         }

//         //     }else if(receivedarray[0] != 2 && receivedarray[0] != 0){
//         //         digitalWrite(CURRENT_LED, LOW);
//         //         analogWrite(MOT_1, drone_1_speed);
//         //         analogWrite(MOT_2, drone_1_speed);
//         //         analogWrite(MOT_3, drone_1_speed);
//         //         analogWrite(MOT_4, drone_1_speed);

//         //     }

//         // // IMU control mode 
//         // }else if(receivedarray[7] != -1000 && receivedarray[8] != -1000 && receivedarray[9] != -1000){
//         //     if(receivedarray[0] == 2 || receivedarray[0] == 0){

//         //     }else if(receivedarray[0] != 2 && receivedarray[0] != 0){

//         //     }
//         // }
        
//     }
// }




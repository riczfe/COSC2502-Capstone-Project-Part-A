// #include <WiFi.h>
// #include <Arduino.h>
// #include "Wire.h"
// #include <ESP32Servo.h>
// #include <Wire.h>
// #include <WiFiServer.h>


// #define POT_PIN 4
// // Define the GPIO pins connected to DRV8833 control pins
// #define DRV8833_IN1_PIN  25  // Example GPIO pin for DRV8833 IN1
// #define DRV8833_IN2_PIN  26  // Example GPIO pin for DRV8833 IN2
// #define DRV8833_IN3_PIN  27  // Example GPIO pin for DRV8833 IN2
// #define DRV8833_IN4_PIN  33  // Example GPIO pin for DRV8833 IN2


// // const char* ssid = "RB_MiniDrone";    // SSID of master ESP32's Wi-Fi network
// // const char* password = "123456789";         // Password for master ESP32's Wi-Fi network
// // const char* serverIP = "192.168.4.1";      // IP address of master ESP32
// // const int serverPort = 80;                 // Port number for server 
// int CtrlPWM;

// // int receivedarray[9];  // Array to store received values from master
// // int slavesendarray[] = {1,2,3,4,5,6,7,8,9};

// // WiFiClient client;

// void setup(){
//     Serial.begin(115200);

//     // pinMode(led, OUTPUT);
//     // digitalWrite(led, HIGH);
    

//     pinMode(POT_PIN, INPUT);
//     // pinMode(6, OUTPUT);
//     // pinMode(8, OUTPUT);

//     // pinMode(0, OUTPUT);
//     // pinMode(1, OUTPUT);
//     // pinMode(2, OUTPUT);
//     // pinMode(3, OUTPUT);
//     // pinMode(4, OUTPUT);

//     pinMode(DRV8833_IN1_PIN, OUTPUT);
//     pinMode(DRV8833_IN2_PIN, OUTPUT);
//     pinMode(DRV8833_IN3_PIN, OUTPUT);
//     pinMode(DRV8833_IN4_PIN, OUTPUT);
    
//     // // Connect to Wi-Fi network
//     // WiFi.begin(ssid, password);

//     // // Wait for Wi-Fi connection
//     // while (WiFi.status() != WL_CONNECTED) {
//     //     delay(1000);
//     //     Serial.print(".");
//     // }

//     // // Warning after successfully connected
//     // Serial.println();
//     // Serial.print("WiFi connected successfully to ");
//     // Serial.println(ssid);
    
// }

// void loop(){

//     // Serial.print("shbhb");
    
//     // digitalWrite(5, HIGH);
//     // digitalWrite(6, HIGH);
//     // digitalWrite(7, HIGH);
//     // digitalWrite(8, HIGH);

//     // digitalWrite(1, HIGH);
//     // digitalWrite(2, HIGH);
//     // digitalWrite(3, HIGH);
//     // digitalWrite(0, HIGH);
//     // digitalWrite(4, LOW);



//     CtrlPWM = map(analogRead(POT_PIN), 0, 4095, 0, 255);
//     Serial.println(CtrlPWM);
//     // // Example: Run motor in one direction
//     analogWrite(DRV8833_IN1_PIN, CtrlPWM);
//     analogWrite(DRV8833_IN2_PIN, CtrlPWM);
//     analogWrite(DRV8833_IN3_PIN, CtrlPWM);
//     analogWrite(DRV8833_IN4_PIN, CtrlPWM);



//     // // Example: Brake (stop) the motor
//     // digitalWrite(DRV8833_IN1_PIN, HIGH);  // Both IN1 and IN2 HIGH brakes the motor
//     // digitalWrite(DRV8833_IN2_PIN, HIGH);
//     // delay(2000);  // Motor brakes for 2 seconds

//     // // Example: Run motor in the other direction
//     // digitalWrite(DRV8833_IN1_PIN, LOW);
//     // digitalWrite(DRV8833_IN2_PIN, HIGH);
//     // delay(2000);  // Motor runs in the other direction for 2 seconds

//     // // Example: Coast (free spin) the motor
//     // digitalWrite(DRV8833_IN1_PIN, LOW);  // Both IN1 and IN2 LOW lets the motor coast
//     // digitalWrite(DRV8833_IN2_PIN, LOW);
//     // delay(2000);  // Motor coasts for 2 seconds


//     // //Ensure client is connected to the server
//     // if (!client.connected()) {
//     //     if (!client.connect(serverIP, serverPort)) {
//     //     Serial.println("Failed to connect to server. Retrying...");
//     //     delay(500);
//     //     return;
//     //     }
//     //     Serial.println("Connected to server");
//     // }

//     // // Check and read data from master ESP32
//     // if (client.available()) {
//     //     client.readBytes((uint8_t*)receivedarray, sizeof(receivedarray));

//     //     // Print received data
//     //     for (int i = 0; i < 9; i++) {
//     //         Serial.print(receivedarray[i]);
//     //         Serial.print("\t");
//     //     }
//     //     Serial.println();
//     // }
// }


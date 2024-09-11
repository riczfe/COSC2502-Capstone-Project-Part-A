// #include <WiFi.h>
// #include <Arduino.h>
// #include "Wire.h"
// #include <ESP32Servo.h>
// #include <Arduino.h>
// #include <Wire.h>
// #include <WiFiServer.h>


// const char* ssid = "RB_MiniDrone";    // SSID of master ESP32's Wi-Fi network
// const char* password = "123456789";         // Password for master ESP32's Wi-Fi network
// const char* serverIP = "192.168.4.1";      // IP address of master ESP32
// const int serverPort = 80;                 // Port number for server 

// int receivedarray[9];  // Array to store received values from master
// int slavesendarray[] = {1,2,3,4,5,6,7,8,9};

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

//     // Check and read data from master ESP32
//     if (client.available()) {
//         client.readBytes((uint8_t*)receivedarray, sizeof(receivedarray));

//         // Print received data
//         for (int i = 0; i < 9; i++) {
//         Serial.print(receivedarray[i]);
//         Serial.print("\t");
//         }
//         Serial.println();

//         // // Send data back to Master
//         // client.write((uint8_t*)slavesendarray, sizeof(slavesendarray));
//     }  
// }




// #include <Arduino.h>
// #include <WiFi.h>

// #define Wifi_Network "Tam"
// #define Wifi_Password "88889999"
// #define Wifi_connecting_time_out 20000

// int potvalue = 2;

// WiFiServer server(80);

// void connect_Wifi(){
//     Serial.print("Connecting to Wifi");
//     WiFi.mode (WIFI_STA);
//     WiFi.begin(Wifi_Network, Wifi_Password);

//     unsigned long startAttemptime = millis();

//     while(WiFi.status() != WL_CONNECTED && millis() - startAttemptime < Wifi_connecting_time_out){
//         Serial.print(".");
//         delay(100);
//     }

//     if(WiFi.status() != WL_CONNECTED){
//         Serial.print("Fail");
//     }else{
//         Serial.println("Connected");
//         Serial.println(WiFi.localIP());
//         server.begin();
//     }
// }

// void setup(){
//     Serial.begin(115200);
//     connect_Wifi();
// }

// void loop(){
//     WiFiClient client = server.available();
//     if(client){
//         Serial.println("New client");
//         String currentline =" ";


//         while(client.connected()){
//             if(client.available()){
//                 char c = client.read();
//                 Serial.print(c);
//                 if(c=='\n'){
//                     if(currentline.length() == 0){
//                         client.println("HTTP/1.1 200 OK");
//                         client.println("Content-type:text/html");
//                         client.println();

//                         client.print("Click <a href=\"/H\">here</a> to turn on the led on pin 5 on.<br>");
//                         client.print("Click <a href=\"/L\">here</a> to turn on the led on pin 5 off.<br>");
//                         client.println();

//                         break;
//                     }else{
//                         currentline = "";
//                     }
//                 }else if (c!='\r'){
//                     currentline += c;
//                 }


//                 if (currentline.endsWith("/H")){
//                     potvalue = 1;
//                     Serial.print("Pot 5 value: ");
//                     Serial.println(potvalue);
                
//                 }else if (currentline.endsWith("/L")){
//                     potvalue = 0;
//                     Serial.print("Pot 5 value: ");
//                     Serial.println(potvalue);
                    
//                 }
//             }
//         }
//         client.stop();
//     }
    

// }
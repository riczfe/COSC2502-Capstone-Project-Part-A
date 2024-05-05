#include <Arduino.h>
#include "Arduino.h"
#include <ArduinoWebsockets.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <TB6612_ESP32.h>
#include <ESP32Servo.h>

#include "config.h"
#include "web.h"

/** ESP32 robot tank with wifi and one joystick web control sketch. 
    Based on SMARS modular robot project using esp32 and tb6612.
    https://www.thingiverse.com/thing:2662828

    for complete complete program: https://github.com/nkmakes/SMARS-esp32

    Made by nkmakes.github.io, August 2020.

    -----------------------------------------
    Camera stream based upon:
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    Adapted by Manos Zeakis for ESP32 and TB6612FNG
*/

Servo ESC; 
#define MOTOR_PIN 13     // Pin attached to the ESC signal pin
#define MIN_SIGNAL 1000  // Minimum PWM signal for ESC
#define MAX_SIGNAL 2000  // Maximum PWM signal for ESC
#define POT_PIN 4        // Pin attached to the potentiometer

void setup() {
    Serial.begin(9600);
    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);

    // Initialize ESC
    ESC.attach(MOTOR_PIN, MIN_SIGNAL, MAX_SIGNAL);
    ESC.writeMicroseconds(MIN_SIGNAL); // Start with the ESC at minimum signal
    delay(2000); // Stable initialization
    ESC.writeMicroseconds(MAX_SIGNAL); // Max calibration signal
    delay(2000);
    ESC.writeMicroseconds(MIN_SIGNAL); // Reset to min

    webserver.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
        AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", index_html_gz, sizeof(index_html_gz));
        response->addHeader("Content-Encoding", "gzip");
        request->send(response);
    });

    webserver.begin();
    server.listen(82);
    Serial.println("Server is live!");
}
 
// handle http messages
void handle_message(WebsocketsMessage msg) {
    if (msg.data().startsWith("POT:")) {
        int sliderValue = msg.data().substring(4).toInt();
        int ESCSignal = map(sliderValue, 0, 180, MIN_SIGNAL, MAX_SIGNAL);
        ESC.writeMicroseconds(ESCSignal);
    } else {
        commaIndex = msg.data().indexOf(',');
        LValue = msg.data().substring(0, commaIndex).toInt();
        RValue = msg.data().substring(commaIndex + 1).toInt();
        motor1.drive(LValue);
        motor2.drive(RValue);
    }
}

 
void loop()
{
  auto client = server.accept();
  if (client.available()) {
    auto msg = client.readBlocking();
    handle_message(msg);
  }
  // Potentiometer control for ESC
  int potValue = analogRead(POT_PIN);
  int CtrlPWM = map(potValue, 0, 4095, 0, 180);
  int ESCSignal = CtrlPWM * 2000 / 180;  // Convert to PWM signal range
  ESC.writeMicroseconds(ESCSignal);
}
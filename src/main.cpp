#include <Arduino.h>
#include "Arduino.h"
#include <ArduinoWebsockets.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <TB6612_ESP32.h>
#include <ESP32Servo.h>

#include "config.h"
#include "web.h"

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
        String sliderValueStr = msg.data().substring(4); // Get the part after "POT:"
        int sliderValue = sliderValueStr.toInt(); // Convert string to integer

        // Map slider value (assuming it's from 0 to 300) to valid PWM signal for ESC
        int ESCSignal = map(sliderValue, 0, 300, MIN_SIGNAL, MAX_SIGNAL);
        ESC.writeMicroseconds(ESCSignal);  // Send the PWM signal to the ESC

        // Print updated value to the serial console
        Serial.print("Slider Value: ");
        Serial.print(sliderValue);
        Serial.print(" - Mapped ESC Signal: ");
        Serial.println(ESCSignal);
    }
}


void loop() {
    auto client = server.accept();
    if (client.available()) {
        auto msg = client.readBlocking();
        handle_message(msg);
    }
    // Potentiometer control for ESC
    int potValue = analogRead(POT_PIN);  // Read the potentiometer value
    int CtrlPWM = map(potValue, 0, 4095, 0, 180);  // Convert to a range suitable for ESC
    int ESCSignal = map(CtrlPWM, 0, 180, MIN_SIGNAL, MAX_SIGNAL);  // Convert to PWM signal range
    ESC.writeMicroseconds(ESCSignal);  // Send the signal to the ESC
    Serial.print("Potentiometer Value: ");
    Serial.print(potValue);
    Serial.print(" - ESC Signal: ");
    Serial.println(ESCSignal);
    delay(100);  // Short delay for stability and readability
}
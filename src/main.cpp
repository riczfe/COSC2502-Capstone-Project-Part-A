#include <Arduino.h>
#include "Arduino.h"
#include <ArduinoWebsockets.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <TB6612_ESP32.h>

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

void setup()
{
  Serial.begin(9600);

  // Create AP
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // HTTP handler assignment
  webserver.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", index_html_gz, sizeof(index_html_gz));
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });

  // start server
  webserver.begin();
  server.listen(82);
  Serial.print("Is server live? ");
  Serial.println(server.available());
 
}
 
// handle http messages
void handle_message(WebsocketsMessage msg) {
  commaIndex = msg.data().indexOf(',');
  LValue = msg.data().substring(0, commaIndex).toInt();
  RValue = msg.data().substring(commaIndex + 1).toInt();
  motor1.drive(LValue);
  motor2.drive(RValue);
}
 
void loop()
{
  auto client = server.accept();
  client.onMessage(handle_message);
  while (client.available()) {
    client.poll();
  }
}
// EspNowCommunication.cpp
#include "EspNowCommunication.h"

struct_msg_Receive Receive_Data;
struct_msg_Sent Sent_Data;

uint8_t broadcastAddress[] = {0xE0, 0x5A, 0x1B, 0xA2, 0x0F, 0x3C};

esp_now_peer_info_t peerInfo;
unsigned long time_prev_serial = 0;

void OnDataReceive(const uint8_t *mac, const uint8_t *incomingData, int len) {
  Serial.print(micros() / 1000);
  Serial.println("\tData received!");
  memcpy(&Receive_Data, incomingData, sizeof(Receive_Data));
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print(micros() / 1000);
  Serial.println("\tData sent!");
}

float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void espnow_initialize() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataReceive);
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void setupEspNow() {
  Serial.begin(115200); // Ensure serial monitor is initialized at your main setup
  espnow_initialize();
}

void handleEspNowCommunication() {
  Sent_Data.Sent_PotAngle = floatMap(Receive_Data.Receive_PotValue, 0, 4095, 0, 300);
  esp_now_send(broadcastAddress, (uint8_t *)&Sent_Data, sizeof(Sent_Data));

  if (micros() - time_prev_serial >= 20000) {
    time_prev_serial = micros();
    SerialDataWrite();
  }
}
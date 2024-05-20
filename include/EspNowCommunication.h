// EspNowCommunication.h
#ifndef ESP_NOW_COMMUNICATION_H
#define ESP_NOW_COMMUNICATION_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

typedef struct {
  int Receive_PotValue;
} struct_msg_Receive;

typedef struct {
  int Sent_PotAngle;
} struct_msg_Sent;

extern struct_msg_Receive Receive_Data;
extern struct_msg_Sent Sent_Data;

void setupEspNow();
void handleEspNowCommunication();
void SerialDataWrite();  // Declare here if it's used in other files.

#endif
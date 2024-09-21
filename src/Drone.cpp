#include <WiFi.h>
#include <Arduino.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <ESP32Servo.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoWebsockets.h>
#include "web.h"
#include "MyPID.h"

#define CURRENT_LED 32    // Potentiometer
#define BUTTON_RED 4      // Button for virtual control method
#define BUTTON_YELLOW 2   // Button for physical control method
#define VIRTUAL_LED 16
#define PHYSICAL_LED 17
#define MAX_CLIENTS 4     // Allow maximum 4 drones connected to controller

#define MOT_1 26
#define MOT_2 33
#define MOT_3 27
#define MOT_4 25

const char* ssid = "RB_MiniDrone";    // SSID of master ESP32's Wi-Fi network
const char* password = "123456789";   // Password for master ESP32's Wi-Fi network

MPU6050 mpu;       
MPU6050 accelgyro;

uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 
Quaternion q;           
VectorFloat gravity;    
float ypr[3];           
int16_t ax, ay, az;     
int16_t gx, gy, gz;     

double anglex, angley, anglez;
float gyrox, gyroy, gyroz;
float accx, accy, accz;

int control_method = 0;
int buttonState_Left = 0, buttonState_Right = 0;
int CtrlPWM = 0;                      
int receivedarray[10];
int total_drone_being_controlled = 2; 

using namespace websockets;
WebsocketsServer server2;
AsyncWebServer webserver(80);
WiFiServer server(80);

WiFiClient clients[MAX_CLIENTS + 1];
bool clientConnected[MAX_CLIENTS + 1] = {false};

// Function Declarations
void Init_MPU();
void Get_MPUangle();
void Get_accelgyro();
void handleMotorControl(int data[]);

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_YELLOW, INPUT_PULLDOWN); 
    pinMode(BUTTON_RED, INPUT_PULLDOWN);
    pinMode(PHYSICAL_LED, OUTPUT);
    pinMode(VIRTUAL_LED, OUTPUT);
    pinMode(CURRENT_LED, OUTPUT);

    pinMode(MOT_1, OUTPUT);
    pinMode(MOT_2, OUTPUT);
    pinMode(MOT_3, OUTPUT);
    pinMode(MOT_4, OUTPUT);

    Init_MPU();
    Init_PID();

    delay(1000);

    // Choose control method
    Serial.println("\nChoose control method");

    while (1) { 
        buttonState_Left = digitalRead(BUTTON_RED);
        if (buttonState_Left == HIGH) {
            control_method = 1;
            break;
        }

        buttonState_Right = digitalRead(BUTTON_YELLOW);
        if (buttonState_Right == HIGH) {
            control_method = 2;
            break;
        }
    }

    // Emit Wi-Fi network
    WiFi.softAP(ssid, password);
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());

    // Virtual control method
    if (control_method == 1) {        
        webserver.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
            AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", index_html_gz, sizeof(index_html_gz));
            response->addHeader("Content-Encoding", "gzip");
            request->send(response);
        });
        digitalWrite(VIRTUAL_LED, HIGH);
        webserver.begin();
        server2.listen(82);
    }
    // Physical control method
    else if (control_method == 2) {
        digitalWrite(PHYSICAL_LED, HIGH);
        server.begin(); 
    }
}

void Init_MPU() {
    Wire.begin(21, 22);   
    Wire.setClock(400000);  
    accelgyro.initialize();  
    mpu.initialize();        
    mpu.dmpInitialize();     
    mpu.setDMPEnabled(true); 
    packetSize = mpu.dmpGetFIFOPacketSize();
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
}

void Get_MPUangle() {
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    while (fifoCount < packetSize)
        fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    anglex = ypr[2] * 180 / M_PI;
    angley = -ypr[1] * 180 / M_PI;
    anglez = -ypr[0] * 180 / M_PI;
}

void Get_accelgyro() {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gyrox = gx / 131.0;
    gyroy = gy / 131.0;
    gyroz = gz / 131.0;
    accx = ax / 16384.;
    accy = ay / 16384.;
    accz = az / 16384.;
}

void loop() {
    if (control_method == 1) {  // Virtual controller method
        if (server2.poll()) {
            WebsocketsClient client = server2.accept();
            if (client.available()) {
                WebsocketsMessage msg = client.readBlocking();  // Use readBlocking with no arguments

                String message = msg.data();  // Get the data from the message

                // Parse received data into array
                for (int i = 0; i < 10; i++) {
                    receivedarray[i] = message[i] - '0';  // Simple parsing, update as needed
                }
                handleMotorControl(receivedarray);
            }
        }
    } 
    else if (control_method == 2) {  // Physical controller method
        if (server.hasClient()) {
            WiFiClient client = server.available();
            clients[0] = client; 
            if (clients[0].available()) {
                clients[0].readBytes((uint8_t*)receivedarray, sizeof(receivedarray));
                handleMotorControl(receivedarray);
            }
        }
    }
}

void handleMotorControl(int data[]) {
    // Update motor values based on received data
    int motor1 = data[1] - data[2] + data[3] + data[4] - data[5];
    int motor2 = data[1] - data[2] - data[3] - data[4] + data[5];
    int motor3 = data[1] + data[2] - data[3] + data[4] - data[5];
    int motor4 = data[1] + data[2] + data[3] - data[4] + data[5];

    analogWrite(MOT_1, motor1);
    analogWrite(MOT_2, motor2);
    analogWrite(MOT_3, motor3);
    analogWrite(MOT_4, motor4);
}

#include <WiFi.h>
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoWebsockets.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "web.h"
#include "MyPID.h"
#include "MyMotorConfig.h"
#include "MyMPU.h"

#define CURRENT_LED 32      // LED for indicating current status
#define BUTTON_RED 4        // Button for selecting virtual control
#define BUTTON_YELLOW 2     // Button for selecting physical control
#define VIRTUAL_LED 16      // LED to indicate virtual control
#define PHYSICAL_LED 17     // LED to indicate physical control
#define MAX_CLIENTS 4       // Allow maximum 4 drones connected to controller

// Wi-Fi credentials
const char* ssid = "RB_MiniDrone";
const char* password = "123456789";

// WebSocket server
using namespace websockets;
WebsocketsServer server;
AsyncWebServer webserver(80);

// Control method variables
int control_method = 0;
int buttonState_Left = 0, buttonState_Right = 0;

// MPU6050 variables
double anglex, angley, anglez;
double gyrox, gyroy, gyroz;
double accx, accy, accz;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
int16_t ax, ay, az;
int16_t gx, gy, gz;

// PID variables
double motor_cmd_x = 0, motor_cmd_y = 0, motor_cmd_z = 0;

// Motor command variables
int motor1_value = 0, motor2_value = 0, motor3_value = 0, motor4_value = 0;

// Function declarations
void handle_message(WebsocketsMessage msg);
void sendEmergencyStop();
void Init_WiFi();
void Init_WebServer();
void Init_WebSocket();
void Init_MPU();
void Get_MPUangle();
void Get_accelgyro();

void setup() {
    Serial.begin(115200);
    // Initialize pins
    pinMode(BUTTON_YELLOW, INPUT_PULLDOWN);
    pinMode(BUTTON_RED, INPUT_PULLDOWN);
    pinMode(PHYSICAL_LED, OUTPUT);
    pinMode(VIRTUAL_LED, OUTPUT);
    pinMode(CURRENT_LED, OUTPUT);

    // Initialize motors
    Init_MotorPin();

    // Initialize MPU6050
    Init_MPU();

    // Initialize PID
    Init_PID();

    delay(1000);

    Serial.println("\nChoose control method");

    while(1){ 
        // Virtual Method (Red Button)
        buttonState_Left = digitalRead(BUTTON_RED);
        if(buttonState_Left == HIGH){
            control_method = 1;
            break;
        }
        // Physical Method (Yellow Button)
        buttonState_Right = digitalRead(BUTTON_YELLOW);
        if(buttonState_Right == HIGH){
            control_method = 2;
            break;
        }
    }

    // Initialize Wi-Fi
    Init_WiFi();

    if(control_method == 1){  // Virtual control
        Serial.println("Virtual control selected");
        digitalWrite(VIRTUAL_LED, HIGH);
        // Initialize Web Server and WebSocket Server
        Init_WebServer();
        Init_WebSocket();
    }
    else if(control_method == 2){  // Physical control
        digitalWrite(PHYSICAL_LED, HIGH);
        Serial.println("Physical control selected");
        // Start TCP server for physical control
        server.listen(82);  // Using port 82 for consistency
        Serial.println("TCP server started for physical control on port 82");
    }
}

void loop() {
    if(control_method == 1){  // Virtual controller
        auto client = server.accept();
        if (client.available()) {
            Serial.println("WebSocket client connected");
            client.onMessage(handle_message);

            while (client.available()) {
                client.poll();

                // Read data from MPU6050
                Get_MPUangle();
                Get_accelgyro();

                // Compute PID
                Compute_PID();

                // Combine motor commands with PID outputs
                // Adjust motor speeds based on received motor values and PID outputs
                int pwm1 = constrain(motor1_value + motor_cmd_x - motor_cmd_y - motor_cmd_z, 0, 255);
                int pwm2 = constrain(motor2_value - motor_cmd_x - motor_cmd_y + motor_cmd_z, 0, 255);
                int pwm3 = constrain(motor3_value - motor_cmd_x + motor_cmd_y - motor_cmd_z, 0, 255);
                int pwm4 = constrain(motor4_value + motor_cmd_x + motor_cmd_y + motor_cmd_z, 0, 255);

                // Write PWM signals to motors
                ledcWrite(PWM_CHA_AIN1, pwm1);
                ledcWrite(PWM_CHA_BIN1, pwm2);
                ledcWrite(PWM_CHA_CIN1, pwm3);
                ledcWrite(PWM_CHA_DIN1, pwm4);

                // For debugging
                Serial.print("PWM1: "); Serial.print(pwm1);
                Serial.print(" PWM2: "); Serial.print(pwm2);
                Serial.print(" PWM3: "); Serial.print(pwm3);
                Serial.print(" PWM4: "); Serial.println(pwm4);

                // Delay for stability
                delay(20);
            }
        }
    }
    else if(control_method == 2){  // Physical controller
        // Implement physical control logic here
        WiFiClient client = server.accept();
        if (client) {
            Serial.println("Physical controller connected");
            while (client.connected()) {
                if (client.available()) {
                    // Read data from the client
                    String data = client.readStringUntil('\n');
                    data.trim();
                    Serial.println("Received data from physical controller: " + data);

                    // Parse received data
                    int commaIndex1 = data.indexOf(',');
                    int commaIndex2 = data.indexOf(',', commaIndex1 + 1);
                    int commaIndex3 = data.indexOf(',', commaIndex2 + 1);

                    if (commaIndex1 != -1 && commaIndex2 != -1 && commaIndex3 != -1) {
                        motor1_value = data.substring(0, commaIndex1).toInt();
                        motor2_value = data.substring(commaIndex1 + 1, commaIndex2).toInt();
                        motor3_value = data.substring(commaIndex2 + 1, commaIndex3).toInt();
                        motor4_value = data.substring(commaIndex3 + 1).toInt();

                        // Constrain the motor values
                        motor1_value = constrain(motor1_value, 0, 255);
                        motor2_value = constrain(motor2_value, 0, 255);
                        motor3_value = constrain(motor3_value, 0, 255);
                        motor4_value = constrain(motor4_value, 0, 255);

                        // Write PWM signals to motors
                        ledcWrite(PWM_CHA_AIN1, motor1_value);
                        ledcWrite(PWM_CHA_BIN1, motor2_value);
                        ledcWrite(PWM_CHA_CIN1, motor3_value);
                        ledcWrite(PWM_CHA_DIN1, motor4_value);

                        // For debugging
                        Serial.print("Motor values: ");
                        Serial.print(motor1_value); Serial.print(", ");
                        Serial.print(motor2_value); Serial.print(", ");
                        Serial.print(motor3_value); Serial.print(", ");
                        Serial.println(motor4_value);
                    }
                }

                // Read data from MPU6050
                Get_MPUangle();
                Get_accelgyro();

                // Compute PID
                Compute_PID();

                // Optionally adjust motor commands with PID outputs
                // This depends on whether you want PID control in physical mode

                // Delay for stability
                delay(20);
            }
            client.stop();
            Serial.println("Physical controller disconnected");
        }
    }
}

// Function to handle incoming WebSocket messages
void handle_message(WebsocketsMessage msg) {
    String data = msg.data();

    // Handle emergency stop command
    if (data == "EMERGENCY_STOP") {
        sendEmergencyStop();
        return;
    }

    // Split the incoming data by commas
    int commaIndex1 = data.indexOf(',');
    int commaIndex2 = data.indexOf(',', commaIndex1 + 1);
    int commaIndex3 = data.indexOf(',', commaIndex2 + 1);

    if (commaIndex1 != -1 && commaIndex2 != -1 && commaIndex3 != -1) {
        motor1_value = data.substring(0, commaIndex1).toInt();
        motor2_value = data.substring(commaIndex1 + 1, commaIndex2).toInt();
        motor3_value = data.substring(commaIndex2 + 1, commaIndex3).toInt();
        motor4_value = data.substring(commaIndex3 + 1).toInt();

        // Map the received values (0-100) to appropriate PWM signals (0-255)
        motor1_value = map(motor1_value, 0, 100, 0, 255);
        motor2_value = map(motor2_value, 0, 100, 0, 255);
        motor3_value = map(motor3_value, 0, 100, 0, 255);
        motor4_value = map(motor4_value, 0, 100, 0, 255);

        // For debugging
        Serial.print("Received motor values: ");
        Serial.print(motor1_value); Serial.print(", ");
        Serial.print(motor2_value); Serial.print(", ");
        Serial.print(motor3_value); Serial.print(", ");
        Serial.println(motor4_value);
    }
    // Handling directional movement commands
    else if (data == "MOVE_UP") {
        // Implement movement logic for moving up
        // Increase motors 1 and 4, decrease motors 2 and 3
        int movement_increment = 20;
        motor1_value += movement_increment;
        motor2_value -= movement_increment;
        motor3_value -= movement_increment;
        motor4_value += movement_increment;

        // Constrain motor values
        motor1_value = constrain(motor1_value, 0, 255);
        motor2_value = constrain(motor2_value, 0, 255);
        motor3_value = constrain(motor3_value, 0, 255);
        motor4_value = constrain(motor4_value, 0, 255);

        Serial.println("MOVE_UP command received");
    }
    else if (data == "MOVE_DOWN") {
        // Implement movement logic for moving down
        // Increase motors 2 and 3, decrease motors 1 and 4
        int movement_increment = 20;
        motor1_value -= movement_increment;
        motor2_value += movement_increment;
        motor3_value += movement_increment;
        motor4_value -= movement_increment;

        // Constrain motor values
        motor1_value = constrain(motor1_value, 0, 255);
        motor2_value = constrain(motor2_value, 0, 255);
        motor3_value = constrain(motor3_value, 0, 255);
        motor4_value = constrain(motor4_value, 0, 255);

        Serial.println("MOVE_DOWN command received");
    }
    else if (data == "MOVE_LEFT") {
        // Implement movement logic for moving left
        // Increase motors 1 and 2, decrease motors 3 and 4
        int movement_increment = 20;
        motor1_value += movement_increment;
        motor2_value += movement_increment;
        motor3_value -= movement_increment;
        motor4_value -= movement_increment;

        // Constrain motor values
        motor1_value = constrain(motor1_value, 0, 255);
        motor2_value = constrain(motor2_value, 0, 255);
        motor3_value = constrain(motor3_value, 0, 255);
        motor4_value = constrain(motor4_value, 0, 255);

        Serial.println("MOVE_LEFT command received");
    }
    else if (data == "MOVE_RIGHT") {
        // Implement movement logic for moving right
        // Increase motors 3 and 4, decrease motors 1 and 2
        int movement_increment = 20;
        motor1_value -= movement_increment;
        motor2_value -= movement_increment;
        motor3_value += movement_increment;
        motor4_value += movement_increment;

        // Constrain motor values
        motor1_value = constrain(motor1_value, 0, 255);
        motor2_value = constrain(motor2_value, 0, 255);
        motor3_value = constrain(motor3_value, 0, 255);
        motor4_value = constrain(motor4_value, 0, 255);

        Serial.println("MOVE_RIGHT command received");
    }
}

void sendEmergencyStop() {
    ledcWrite(PWM_CHA_AIN1, 0);
    ledcWrite(PWM_CHA_BIN1, 0);
    ledcWrite(PWM_CHA_CIN1, 0);
    ledcWrite(PWM_CHA_DIN1, 0);
    motor1_value = 0;
    motor2_value = 0;
    motor3_value = 0;
    motor4_value = 0;
    Serial.println("Emergency Stop Activated!");
}

void Init_WiFi() {
    WiFi.softAP(ssid, password);
    Serial.println("\nESP32 is now running as an access point.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());
}

void Init_WebServer() {
    webserver.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", index_html_gz, sizeof(index_html_gz));
        response->addHeader("Content-Encoding", "gzip");
        request->send(response);
    });
    webserver.begin();
    Serial.println("Web server started");
}

void Init_WebSocket() {
    server.listen(82);
    Serial.println("WebSocket server started on port 82");
}

void Init_MPU() {
    Wire.begin(21, 22);      // Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);   // Set the SCL clock to 400KHz
    accelgyro.initialize();  // Initialize the accelgyro
    mpu.initialize();        // Initialize the MPU
    mpu.dmpInitialize();     // Initialize the DMP
    mpu.setDMPEnabled(true); // Enable the DMP
    packetSize = mpu.dmpGetFIFOPacketSize();
    mpu.CalibrateAccel(6);   // Calibrate the accelerometer
    mpu.CalibrateGyro(6);    // Calibrate the gyroscope
}

void Get_MPUangle() {
    // Clear buffer
    mpu.resetFIFO();
    // Get FIFO count
    fifoCount = mpu.getFIFOCount();
    // Wait for the FIFO to be filled with the correct data number
    while (fifoCount < packetSize)
        fifoCount = mpu.getFIFOCount();
    // Read a packet from FIFO
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
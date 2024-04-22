// #include <Arduino.h>
// #include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps20.h"
// #include "Wire.h"
// #include <ESP32Servo.h>

// // ================================================================
// // Variable declaration
// // ================================================================
// MPU6050 mpu;       // Prepare the mpu object to obtain the angles from the DMP
// MPU6050 accelgyro; // Prepare the accelgyro object to obtain the gyroscope and the acceleration data

// // MPU variable
// uint16_t packetSize;    // DMP packet size. Default is 42 bytes.
// uint16_t fifoCount;     // count of all bytes currently in FIFO
// uint8_t fifoBuffer[64]; // FIFO storage buffer
// Quaternion q;           // [w, x, y, z]         quaternion container
// VectorFloat gravity;    // [x, y, z]            gravity vector
// float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll
// int16_t ax, ay, az;     // Raw acceleration data from the MPU
// int16_t gx, gy, gz;     // Raw gyroscope data from the MPU

// float anglex, angley, anglez; // angle in the x, y, z direction
// float gyrox, gyroy, gyroz;    // angle rate in the x, y, z direction
// float accx, accy, accz;       // acceleration in the x, y, z direction

// unsigned long time_prev = 0; // data for the serial communication
// // Potentiomer
// #define MAX_SIGNAL 2000 // Parameter required for the ESC definition
// #define MIN_SIGNAL 1000 // Parameter required for the ESC definition
// #define MOTOR_PIN 13    // Pin 13 attached to the ESC signal pin
// #define POT_PIN 4       // Pin 4 attached to the potentiometer

// Servo ESC;                   // Define the ESC
// int CtrlPWM;                 // Control Signal. Varies between [0 - 180]


// // ================================================================
// // Function Declaration
// // ================================================================
// void Init_Serial();     // Function to init the serial monitor
// void Init_MPU();        // Function to init the MPU6050
// void Get_MPUangle();    // Function to get the angle from the MPU6050
// void Get_accelgyro();   // Function to get the gyro and acc from the MPU6050
// void SerialDataPrint(); // Function to print data on the serial monitor
// void SerialDataPrintPotentiometer(); // Function to print data on the serial monitor
// void Init_ESC();        // Function to init the ESC

// // ================================================================
// // Setup function
// // ================================================================
// void setup()
// {
//   Init_Serial();
//   Init_MPU();
// }

// // ================================================================
// // Loop function
// // ================================================================
// void loop()
// {
//   CtrlPWM = map(analogRead(POT_PIN), 0, 4095, 0, 180); // Read the pot, map the reading from [0, 4095] to [0, 180]
//   ESC.write(CtrlPWM);                                  // Send the command to the ESC 
//   Get_MPUangle();
//   Get_accelgyro();
//   SerialDataPrint();

// }

// // ================================================================
// // Function Definition
// // ================================================================
// void Init_Serial()
// {
//   Serial.begin(115200);
//   while (!Serial)
//     ;
// }
// // ================================================================
// void Init_MPU()
// {
//   Wire.begin(21, 22);      // Wire.begin(I2C_SDA, I2C_SCL);
//   Wire.setClock(400000);   // Set the SCL clock to 400KHz
//   accelgyro.initialize();  // Initialize the accelgyro
//   mpu.initialize();        // Initialize the MPU
//   mpu.dmpInitialize();     // Initialize the DMP (microchip that calculate the angle on the MPU6050 module)
//   mpu.setDMPEnabled(true); // Enable the DMP
//   packetSize = mpu.dmpGetFIFOPacketSize();
//   mpu.CalibrateAccel(6); // Calibrate the accelerometer
//   mpu.CalibrateGyro(6);  // Calibrate the gyroscope
// }
// // ================================================================
// void Init_ESC()
// {
//   ESC.attach(MOTOR_PIN, MIN_SIGNAL, MAX_SIGNAL);
//   ESC.writeMicroseconds(MIN_SIGNAL);
// }
// // ================================================================
// void Get_MPUangle()
// {
//   // Clear buffer
//   mpu.resetFIFO();
//   // Get FIFO count
//   fifoCount = mpu.getFIFOCount();
//   // Wait for the FIFO to be filled with the correct data number
//   while (fifoCount < packetSize)
//     fifoCount = mpu.getFIFOCount();
//   // read a packet from FIFO
//   mpu.getFIFOBytes(fifoBuffer, packetSize);
//   mpu.dmpGetQuaternion(&q, fifoBuffer);
//   mpu.dmpGetGravity(&gravity, &q);
//   mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//   anglex = ypr[2] * 180 / M_PI;
//   angley = -ypr[1] * 180 / M_PI;
//   anglez = -ypr[0] * 180 / M_PI;
// }
// // ================================================================
// void Get_accelgyro()
// {
//   accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//   gyrox = gx / 131.0;
//   gyroy = gy / 131.0;
//   gyroz = gz / 131.0;
//   accx = ax / 16384.;
//   accy = ay / 16384.;
//   accz = az / 16384.;
// }
// // ================================================================
// void SerialDataPrint()
// {
//   if (micros() - time_prev >= 1000){
//     time_prev = micros();
//     Serial.print(millis());
//     Serial.print("\t");
//     Serial.print(anglex);
//     Serial.print("\t");
//     Serial.print(angley);
//     Serial.print("\t");
//     Serial.print(anglez);
//     Serial.print("\t");
//     Serial.print(gyrox);
//     Serial.print("\t");
//     Serial.print(gyroy);
//     Serial.print("\t");
//     Serial.print(gyroz);
//     Serial.print("\t");
//     Serial.print(CtrlPWM);
//     // Serial.print("\t");

//     // Serial.print(anglex); //desired yaw pitch roll
//     // Serial.print("\t");
//     // Serial.print(angley);
//     // Serial.print("\t");
//     // Serial.print(anglez);
//     // Serial.print("\t");
//     // Serial.print(CtrlPWM);
//     // Serial.print("\t");
//     // Serial.print(CtrlPWM);
//     // Serial.print("\t");
//     // Serial.print(CtrlPWM);
//     // Serial.print("\t");
//     // Serial.print(CtrlPWM);
//     Serial.print("\n");
//   }
// }


#include <Arduino.h>
#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"

// ================================================================
// Variable declaration
// ================================================================
MPU6050 mpu;
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

uint16_t packetSize;    // DMP packet size. Default is 42 bytes.
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;                  // [w, x, y, z]         quaternion container
VectorFloat gravity;           // [x, y, z]            gravity vector
float ypr[3];                  // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
double anglex, angley, anglez; // x, y, z angular position
double gyrox, gyroy, gyroz;    // x, y, z angular velocity
double accx, accy, accz;       // x, y, z linear acceleration

// ================================================================
// Function Definition
// ================================================================

void Init_MPU()
{
  Wire.begin(4, 16);       // Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);   // Set the SCL clock to 400KHz
  accelgyro.initialize();  // Initialize the accelgyro
  mpu.initialize();        // Initialize the MPU
  mpu.dmpInitialize();     // Initialize the DMP (microchip that calculate the angle on the MPU6050 module)
  mpu.setDMPEnabled(true); // Enable the DMP
  packetSize = mpu.dmpGetFIFOPacketSize();
  mpu.CalibrateAccel(6); // Calibrate the accelerometer
  mpu.CalibrateGyro(6);  // Calibrate the gyroscope
}
// ================================================================
void Get_MPUangle()
{
  // Clear buffer
  mpu.resetFIFO();
  // Get FIFO count
  fifoCount = mpu.getFIFOCount();
  // Wait for the FIFO to be filled with the correct data number
  while (fifoCount < packetSize)
    fifoCount = mpu.getFIFOCount();
  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  anglex = ypr[2] * 180 / M_PI;
  angley = -ypr[1] * 180 / M_PI;
  anglez = -ypr[0] * 180 / M_PI;
}
// ================================================================
void Get_accelgyro()
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  gyrox = gx / 131.0;
  gyroy = gy / 131.0;
  gyroz = gz / 131.0;
  accx = ax / 16384.;
  accy = ay / 16384.;
  accz = az / 16384.;
}

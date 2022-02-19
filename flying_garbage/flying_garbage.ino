#include <Wire.h>                   // 
#include <SoftwareSerial.h>         // HC-05 Bluetooth
#include <MPU6050.h>                // MPU6050 IMU


// Init Bluetooth Module for serial pins RX (2) and TX (3)
const byte rxPin = 2;
const byte txPin = 3;
SoftwareSerial btModule(rxPin, txPin);


// Init MPU6050 IMU
MPU6050 mpu;
#define MPU_SDA A4
#define MPU_SCL A5


void setup() {

  // Init serial baud rate and bluetooth baud rate 
  // 115200 is chosen for slightly faster reception/transmission speeds
  Serial.begin(115200);
  btModule.begin(115200);

  if (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G) {
    Serial.println("Could not find a valid MPU6050 sensor.");
    delay(500);
  }
  
}

void loop() {

}

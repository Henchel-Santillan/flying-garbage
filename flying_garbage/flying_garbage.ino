#include <Wire.h>                   // 
#include <Vector.h>                 // Container similar to c++ std::vector
#include <SoftwareSerial.h>         // HC-05 Bluetooth
#include <MPU6050.h>                // MPU6050 IMU



//-------- HC-05 BLUETOOTH --------//

// Init Bluetooth Module for serial pins RX (2) and TX (3)
const byte rxPin = 2;
const byte txPin = 3;
SoftwareSerial btModule(rxPin, txPin);

int flag = 0;


//-------- MPU6050 IMU --------//

// Init MPU6050 IMU and define analog pins for I2C
MPU6050 mpu;
#define MPU_SDA A4  // data line
#define MPU_SCL A5  // clock line

// TODO: Add MPU6050 I2C Address (?)

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;

#define BAUD_RATE 115200

void setup() {

  // Pin Definitions
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);

  // Init serial baud rate and bluetooth baud rate 
  // 115200 is chosen for slightly faster reception/transmission speeds
  Serial.begin(BAUD_RATE);
  btModule.begin(BAUD_RATE);

  // Init MPU6050 and validate
  if (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G) {
    Serial.println("Could not find a valid MPU6050 sensor.");
    delay(500);
  }
  
}

void loop() {

  // Reads module if bytes available for reading > 0
  if (btModule.available() > 0) {
    flag = btModule.read();
  }
}

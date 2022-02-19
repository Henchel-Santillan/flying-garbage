// BOARD: UNO R3

#include <Wire.h>                   // Allows communication with I2C / TWI devices. Here, MPU is I2C.
#include <Vector.h>                 // Container similar to c++ std::vector
#include <Servo.h>                  // Used to represent the Electronic Speed Controllers (ESCs)
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


//-------- I2C --------//
byte recvFromI2C;
unsigned long startTimeI2C;


//-------- ESC --------//
Servo ESC_TOP;    // ESC for the top rotor / blade
Servo ESC_TAIL;   // ESC for the tail rotor / blade


//-------- OTHER CONSTANTS AND SETUP --------//

#define BAUD_RATE 115200
#define MIN_PULSE_WIDTH 1000
#define MAX_PULSE_WIDTH 2000
#define ESC_TOP_PIN 9
#define ESC_TAIL_PIN 8


void setup() {

  // Pin Definitions for the bluetooth module (rx == receiver == input, tx == transmitter == output)
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);

  // Init serial baud rate and bluetooth baud rate 
  // 115200 is chosen for slightly faster reception/transmission speeds
  Serial.begin(BAUD_RATE);
  while (!Serial);
  btModule.begin(BAUD_RATE);

  // Attach the ESCs to their respective pins and specify the minimum and maximum pulse widths
  ESC_TOP.attach(ESC_TOP_PIN, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  ESC_TAIL.attach(ESC_TAIL_PIN, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

  // Init Wire (join I2C bus)
  Wire.begin();

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

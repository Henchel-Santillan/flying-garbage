// BOARD: UNO R3

#include <math.h>
#include <Wire.h>                   // Allows communication with I2C / TWI devices. Here, MPU is I2C.
#include <Vector.h>                 // Container similar to c++ std::vector
#include <Servo.h>                  // Used to represent the Electronic Speed Controllers (ESCs)
#include <SoftwareSerial.h>         // HC-05 Bluetooth
#include <MPU6050.h>                // MPU6050 IMU


//-------- USEFUL CONSTANTS AND SETUP --------//

#define BAUD_RATE 115200
#define MIN_PULSE_WIDTH 1000
#define MAX_PULSE_WIDTH 2000
#define ESC_TOP_PIN 9
#define ESC_TAIL_PIN 8

// First 1000 IMU readings to generate error stats
#define SAMPLE 1000
#define G 9.81


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
const int MPU_I2C_ADDR = 0x68 // If AD0 on the breakout is LOW (0)

// Raw data from accelerometer, gyroscope
float accX, accY, accZ;
float temp;
float gyroX, gyroY, gyroZ;

// Error values for accelerometer readings (A) and gyroscope (G), respectively
float errAX = 0, errAY = 0, errAZ = 0;
float errGX = 0, errGY = 0, errGZ = 0;

// Angle information to be computed from raw data
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;

// Other angle + timing information
float phiFOld = 0, phiFNew, thetaFOld = 0, thetaFNew;
float tElapsed, tCurrent = 0, tPrevious;


//-------- ESC --------//

#define ESC_HIGH_DEF 200
#define ESC_LOW_DEF 20

Servo ESCTop;    // ESC for the top rotor / blade
Servo ESCTail;   // ESC for the tail rotor / blade

typedef struct MotorType {
  Servo esc;
  int pin;
};

typedef struct ESCSettingsType {
  int low;
  int high;
};

int step = 10;
int currentSpeed;

MotorType motorTail;
MotorType motorTop;

ESCSettingsType ESCSettings;


//-------- HELPER FUNCTIONS --------//

float roll(const float aX, const float aY, const float aZ) {
  return ( (atan( accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * 180 / PI );
}

float pitch(const float aX, const float aY, const float aZ) {
  return ( atan( (-1 * accX) / sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / PI );
}

// Computes the error of the IMU
void computeErrorIMU() {
  // 
  
  int count = 0;

  // Accelerometer summataion
  while (c < SAMPLE) {
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_I2C_ADDR, 6, true);  // next 6 registers (from 0x3B --> 0x42)
    while (Wire.available() < 6);

    accX = Wire.read() << 8 | Wire.read();
    accY = Wire.read() << 8 | Wire.read();
    accZ = Wire.read() << 8 | Wire.read();

    errAX += accX * G;
    errAY += accY * G;
    errAZ += accZ * G;

    count++;
  }

  errAX = errAX / SAMPLE;
  errAY = errAY / SAMPLE;
  errAZ / errAZ / SAMPLE;

  count = 0;
  while (c < SAMPLE) {
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_I2C_ADDR, 6, true);  // next 6 registers (from 0x3B --> 0x42)
    while (Wire.available() < 6);
    
    accX = Wire.read() << 8 | Wire.read();
    accY = Wire.read() << 8 | Wire.read();
    accZ = Wire.read() << 8 | Wire.read();
    
    errAX += roll(accX, accY, accZ);
    errAY += pitch(accX, accY, accZ);
    
    count++;
  }

  count = 0;

  // Gyroscope summation
  while (c < SAMPLE) {
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_I2C_ADDR. 6, true);  // from 0x43 --> 0x48
    while (Wire.available() < 6);

    gyroX = Wire.read() << 8 | Wire.read();
    gyroY = Wire.read() << 8 | Wire.read();
    gyroZ = Wire.read() << 8 | Wire.read(); 

    errGX += gyroX;
    errGY += gyroY;
    errGZ += gyroZ;

    count++;
  }

  errGX = errGX / SAMPLE;
  errGY = errGY / SAMPLE;
  errGZ = errGZ / SAMPLE;
}



void setup() {

  // Pin Definitions for the bluetooth module (rx == receiver == input, tx == transmitter == output)
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);

  // Init serial baud rate 
  // 115200 is chosen for slightly faster reception/transmission speeds
  Serial.begin(BAUD_RATE);
  while (!Serial);

  // Init Wire (join I2C bus)
  Wire.begin();
  
  // Init comms with MPU6050 (to register 6B, and place a 0, i.e. reset, in the register)
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(0x6B); // requested starting register
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Configure the accelerometer
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG register
  Wire.write(0x10); // +/- 8g full scale range
  Wire.endTransmission(true);

  // Set the sample errors for accelerometer and gyroscope
  computeErrorIMU();

  // Init bluetooth baud rate
  btModule.begin(BAUD_RATE);

  // Attach the ESCs to their respective pins and specify the minimum and maximum pulse widths
  ESCTop.attach(ESC_TOP_PIN, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  ESCTail.attach(ESC_TAIL_PIN, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

  // Tie ESCs and pins to convenience structs
  motorTail.esc = ESCTop;
  motorTail.pin = ESC_TOP_PIN;
  motorTop.esc = ESCTail;
  motorTop.pin = ESC_TAIL_PIN;

  // Init the ESCSettingsType
  ESCSettings.low = ESC_LOW_DEFAULT;
  ESCSettings.high = ESC_HIGH_DEFAULT;

  // Init MPU6050 and validate
  if (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G) {
    Serial.println("Could not find a valid MPU6050 sensor.");
    delay(500);
  }

  // Give time for full initialization of components
  delay(500);
}


void loop() {
  // Read accelerometer data IN
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(0x3B); // 0x3B register is ACCEL_XOUT_H, which contains the 8 most significant bits for accelerometer readings)
  Wire.endTransmission(false);  // Keep connection active, since Arduino will send restart.
  Wire.requestFrom(MPU_I2C_ADDR, 14, true); // Request a total of 14 registers
  while (Wire.available() < 6);
  
  // Conv 2 8-bit (1 byte) reads into 1 16-bit output (left shift with 8 or 2^3, and bitwise OR with next read)
  // Need this since accelerometer values are 16 bit (2 bytes), and Wire.read() only does 1 byte at a time
  accX = Wire.read() << 8 | Wire.read();
  accY = Wire.read() << 8 | Wire.read();
  accZ = Wire.read() << 8 | Wire.read();

  accAngleX = roll(accX, accY, accZ) - errAX;
  accAngleY = pitch(accX, accY, accZ) - errAY;

  // Low pass filter for accelerometer data
  phiFNew = 0.9 * phiFOld + 0.1 * accAngleX;
  thetaFNew = 0.9 * thetaFOld + 0.1 * accAngleY;

  tPrevious = tCurrent;
  tCurrent = millis();
  tElapsed = (tCurrent - tPrevious) / 1000; // div by 1000 since millis() was used --> get seconds

  
  // Read in the gyroscope data IN
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_I2C_ADDR, 6, true);
  while (Wire.available() < 6);
  
  gyroX = (Wire.read() << 8 | Wire.read()) - errGX;
  gyroY = (Wire.read() << 8 | Wire.read()) - errGY;
  gyroZ = (Wire.read() << 8 | Wire.read()) - errGZ;

  // Complementary filter (high-pass filter): combine raw gyro angle data with low filter for accelerometer data
  gyroAngleX = 0.96 * (gyroAngleX + gyroX * elapsedTime) + 0.04 * phiFNew;  // deg/s * s = deg
  gyroAngleY = 0.96 * (gyroAngleY + gyroY * elapsedTime) + 0.04 * phiFNew;

  roll = gyroAngleX;
  pitch = gyroAngleY;
  yaw = yaw + gyroZ * elapsedTime;

  Serial.print("Roll:");
  Serial.println(roll);

  Serial.print("Pitch:");
  Serial.println(pitch);

  Serial.print("Yaw:");
  Serial.println(yaw);

  // Update the low filter values for accelerometer values
  phiFOld = phiFNew;
  thetaFold = thetaFNew;

  delay(10);
  
  // Reads bluetooth module if bytes available for reading > 0
  if (btModule.available() > 0) {
    flag = btModule.read();
  }
}

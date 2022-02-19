#include "SoftwareSerial.h"

// Init Bluetooth Module for serial pins RX (2) and TX (3)
SoftwareSerial BTModule(2, 3);

void setup() {

  // Init serial baud rate and bluetooth baud rate
  Serial.begin(9600);
  BTModule.begin(9600);
  
  
}

void loop() {
  // put your main code here, to run repeatedly:

}

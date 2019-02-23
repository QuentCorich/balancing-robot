/*
 * Authors: Quentin Corich, Austin Hutchison, Alec Richey, Braeden Lieberman
 * 
 * Code for slave controller
 * 
 * sources:
 * http://stackoverflow.com/questions/3991478/building-a-32bit-float-out-of-its-4-composite-bytes-c
 * https://developer.mbed.org/forum/helloworld/topic/2053/
 * https://www.arduino.cc/en/Reference/Wire
 */

/*
 * UNO:
 * A4 - SDA
 * A5 - SCL
 * 
 * MEGA 2560:
 * 20 - SDA
 * 21 - SCL
 */
#include <Wire.h> 

#define address 5

void setup() {
  //initilize I2C
  Wire.begin(address); //Begin i2c
  Wire.onReceive(receiveFloat);
  Serial.begin(9600);
}

void loop() {
}

String receiveData(int howMany) {
  String data = null;
  char datum;
  while (Wire.available())//loops until all bytes are read
    data += datum.Wire.read();

  return data;
}


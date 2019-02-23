/*
 * Authors: Quentin Corich, Austin Hutchison, Alec Richey, Braeden Lieberman
 * 
 * Code for Master Controller
 * 
 * sources: 
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

#define slave_address 5

void setup() {
  Serial.begin(9600);  //set baud rate for serial communication between computer and Arduino
  Wire.begin();        // Begin i2c
}

void loop(){
  
}

void sendChar(int address, char datum){
  Wire.beginTransmission(address);   // Send to device of corresponding address
  Wire.write(datum);//Sends float as a series of 4 bytes
  Wire.endTransmission();      // Stop Transmitting
}

void sendString(int address, String data){
  char datums[sizeof(data) - 1];
  data.toCharArray(datums, sizeof(data) - 1);
  Wire.beginTransmission(address);
  for(int i = 0; i < sizeof(datums) / sizeof(char); i++){
    Wire.write(datums[i]);
  }
  Wire.endTransmission();
}




// Author: Austin Hutchison
// Referances: Bryan Thompson http://www.madebymarket.com/blog/dev/getting-started-with-nrf24L01-and-arduino.html
//             J. Coliz <maniacbug@ymail.com>  https://maniacbug.wordpress.com/2011/11/02/getting-started-rf24/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>
#include "printf.h"

// Hook up the wireless transeiver
//int vcc = 3.3v
//int GND = gnd
int CE = 9;         // This can be any digital pin, change as needed
int CSN = 10;       // This can be any digital pin, change as needed
//int MOSI = 11;
//int MISO = 12;
//int SCK = 13;

// Intialize transeiver
RF24 wireless(CE, CSN);



// Intialize transmit packet
uint8_t transmitData[1];    // Transmit 1 byte packet

//Setup Recieve Packet
typedef union
{
 int number;
 uint8_t bytes[2];
} intOrByte;

uint8_t recieveData[3];    // Recieve 3 byte packet
int go = 0;                // 0=stop; 1=fwd; 2=bck
intOrByte goSpeed;


// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = {0xe7e7e7e7e7LL, 0xc2c2c2c2c2LL};


void setup() {
  Serial.begin(9600);
  printf_begin();
 // Setup wireless
  wireless.begin();
  wireless.openWritingPipe(pipes[1]);
  wireless.openReadingPipe(1,pipes[0]);
  wireless.startListening();
  wireless.printDetails();
}

void loop() {

// Receive Code
  if (wireless.available()){
    Serial.println("***************************************");
    bool done = false;
    while(!done){
      done = wireless.read( &recieveData, sizeof(recieveData) );
    }
  
  Serial.println("Data Received: ");
  
  //Decode wireless data
  go = recieveData[0];
  for (int i = 0; i < 2; i++){
    goSpeed.bytes[i] = recieveData[i+1];
  }
  
  // Serial Print data recieved (testing purposes only
  Serial.print("Go? 0=stop, 1=fw,  2=back    :  ");
  Serial.println(go);
  Serial.print("Speed:  ");
  Serial.println(goSpeed.number);
  

  //Copy rx data packet into transmit data packet
  transmitData[0] = recieveData[0];
  
  

    // Transmit Code  (transmit back to indicate packet recieved)
    wireless.stopListening();
    wireless.write( &transmitData, sizeof(transmitData) );
    Serial.println("Sent response.");
    wireless.startListening();
  }
}

// Author: Austin Hutchison
// Referances: Bryan Thompson http://www.madebymarket.com/blog/dev/getting-started-with-nrf24L01-and-arduino.html
//             J. Coliz <maniacbug@ymail.com>  https://maniacbug.wordpress.com/2011/11/02/getting-started-rf24/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>
#include "printf.h"

// Hook up the wireless transeiver
// vcc = 3.3v
// GND = gnd
int CE = 9;
int CSN = 10;
//int MOSI = 11;
//int MISO = 12;
//int SCK = 13;

// Intialize transeiver
RF24 wireless(CE, CSN);

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
  if (wireless.available()){
    Serial.println("***************************************");
    uint8_t data[4];    // Recieve 4 byte packet
    bool done = false;
    while(!done){
      done = wireless.read( &data, sizeof(data) );
      printf("Got payload @ %lu...\r\n", millis());
  }

  // echo it back real fast
    wireless.stopListening();
    wireless.write( &data, sizeof(data) );
    Serial.println("Sent response.");
    wireless.startListening();

    // do stuff with the data we got.
    Serial.print("First Value: ");
    Serial.println(data[0]);
  }
}

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

// Hook up test led
int LED = 2;

// Intialize transeiver
RF24 wireless(CE, CSN);

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = {0xe7e7e7e7e7LL, 0xc2c2c2c2c2LL};


void setup() {
  Serial.begin(9600);
  printf_begin();
 // Setup wireless
  wireless.begin();
  wireless.openWritingPipe(pipes[0]);
  wireless.openReadingPipe(1,pipes[1]);
  wireless.startListening();
  wireless.printDetails();

  pinMode(LED, OUTPUT);
}

void loop() {
  unsigned long time = millis();
  uint8_t data[4];    // Send 4 byte packet
  data[0] = 15;
  
  // Send data
  wireless.stopListening();
  wireless.write(&data, sizeof(data));
  wireless.startListening();

  // Testing
  // listen for acknowledgement from the receiver
  unsigned long started_waiting_at = millis();
  bool timeout = false;
  while (!wireless.available() && ! timeout)
    if (millis() - started_waiting_at > 250 )
      timeout = true;

  if (timeout){
    Serial.println("Failed, response timed out.");
  } else {
    // the receiver is just going to spit the data back
    wireless.read( &data, sizeof(data) );
    digitalWrite(LED, HIGH);
    delay(100);  // light up the LED for 100ms if it worked.
    digitalWrite(LED, LOW);
    Serial.print("Got response, round trip delay: ");
    Serial.print(millis() - started_waiting_at);
  }

  delay(1000); // wait a second and do it again. 

  
}

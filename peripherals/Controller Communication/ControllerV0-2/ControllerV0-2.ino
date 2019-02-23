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

//Hook up go switch
int fwdToggle = 3;
int bckToggle = 4;
uint8_t go = 0;   // 0 = stop; 1=fwd; 2=bck

//Hook up speed potentiometer
int speedPot = 2;     // Hook up pot to analog pin 2
typedef union
{
 int number;
 uint8_t bytes[2];
} intOrByte;
intOrByte goSpeed;



// Intialize transeiver
RF24 wireless(CE, CSN);

// Intialize transmit packet
uint8_t transmitData[3];    // Send 3 byte packet

// Intialize receive packet
uint8_t receiveData[1];    // Receive 1 byte packet


// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = {0xe7e7e7e7e7LL, 0xc2c2c2c2c2LL};

// Function to read the pot and average x readings
int potRead(int analogPin) {
  int readTimes = 4;
  int readPot[readTimes];    // Stores "readTimes" number of pot reads
  int totalPot = 0;             // Stores the sum of readPot
  int avgPot = 511;         // Stores the avergae of the pot readings
  for (int i = 0; i<readTimes; i++){
     readPot[i] = analogRead(analogPin);
  }
  for (int i = 0; i< readTimes; i++){
    totalPot = totalPot + readPot[i];
  }
  avgPot = totalPot/readTimes;
  return avgPot;
}

// Function to read toggle and return 0=stop; 1=fwd; 2=bck
int toggleRead(int toggleOne, int toggleTwo){ 
  int one = digitalRead(toggleOne);
  int two = digitalRead(toggleTwo);
  if ((one == 0) && (two == 0)){            //Return 0 if both toggles off
    return 0;
  } else if ((one == 1) && (two == 0)){     // Return 1 if toggle one is on
    return 1;
  } else if ((one == 0) && (two == 1)) {    // Return 2 if toggle two is on
    return 2;
  } else {
    return 0;                               // Return zero if both on
  }
}


void setup() {
  Serial.begin(9600);
  printf_begin();
  
 // Setup wireless
  wireless.begin();
  wireless.openWritingPipe(pipes[0]);
  wireless.openReadingPipe(1,pipes[1]);
  wireless.startListening();
  wireless.printDetails();
  transmitData[0] = 0;

  //Setup digital pins
  pinMode(LED, OUTPUT);
  pinMode(fwdToggle, INPUT);
  pinMode(bckToggle, INPUT);
}

void loop() {
  unsigned long time = millis();

  //Transmit if toggle or pot has changed position
  if ((go != toggleRead(fwdToggle, bckToggle)) || ((goSpeed.number <= (potRead(speedPot)-2)) || (goSpeed.number >= (potRead(speedPot)+2)))){
    go = toggleRead(fwdToggle, bckToggle);
    transmitData[0] = go;
    goSpeed.number = potRead(speedPot);    // Get speed value from pot
    for (int i = 0; i< 2; i++){
      transmitData[i+1] = goSpeed.bytes[i];
    }
    

    // Send data
    wireless.stopListening();
    wireless.write(&transmitData, sizeof(transmitData));
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
    wireless.read( &receiveData, sizeof(receiveData) );
      digitalWrite(LED, HIGH);
       // Print data
    Serial.println("*******************************************");
    Serial.print("Got response: ");
    Serial.println(receiveData[0]);
    digitalWrite(LED, LOW);
  }
  }


  delay(20); // wait a 20 milliseconds and look again. 

  
}

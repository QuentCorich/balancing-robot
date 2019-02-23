
// Author: Austin Hutchison
// Referances: Bryan Thompson http://www.madebymarket.com/blog/dev/getting-started-with-nrf24L01-and-arduino.html
//             J. Coliz <maniacbug@ymail.com>  https://maniacbug.wordpress.com/2011/11/02/getting-started-rf24/



#include <LiquidCrystal.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>
#include "printf.h"


typedef union
{
 int number;
 uint8_t bytes[2];
} intOrByte;


// Hook up the wireless transeiver
// vcc = 3.3v   (v on board, whoops. Try to fix)
// GND = gnd
int CE = 9;
int CSN = 10;
//int MOSI = 11;
//int MISO = 12;
//int SCK = 13;
// Intialize transeiver
RF24 wireless(CE, CSN);

// Intiate Toggles PIns
int swOne = 19;
int swTwo = 18;
int swThree = 0;
int swFour = 1; // This needs to be patched on board
int swFive = 2;

// Toggle value storage
uint8_t swOneSave = 0;
uint8_t swTwoSave = 0;
uint8_t swThreeSave = 0;
uint8_t swFourSave = 0;
uint8_t swFiveSave = 0;
uint8_t go = 0;   // 0 = stop; 1=fwd; 2=bck



//Intiate potentiometers Pins
int speedPot = 0;     // Vertical Slide Pot(R1)
int steerPot = 1;     // Horizontal Slide Pot (R2)
int leftPot = 2;      // Left rotary pot (R3)
int rightPot = 3;     // Right rotary pot (R4)

// Pot value saves
intOrByte speedPotSave;
intOrByte steerPotSave;
intOrByte leftPotSave;
intOrByte rightPotSave;



// Intiate LCD Pins
int D4 = 6;
int D5 = 5;
int D6 = 4;
int D7 = 3;
int RegSelect = 8;
int Enable = 7;
//Intialize lcd
LiquidCrystal lcd(8, 7, 6, 5, 4, 3);


// Intialize transmit packet
uint8_t transmitData[5];    // Send 3 byte packet

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
int goToggleEncode(int one, int two){ 
  if ((one == 0) && (two == 0)){            //Return 0 if Toggle 1 is off
    return 0;
  } else if ((one == 0) && (two == 1)){     // Return 0 if Toggle 1 is off
    return 0;
  } else if ((one == 1) && (two == 0)) {    // Return 2 if toggle two is off
    return 2;
  } else if ((one == 1) && (two == 1)){
    return 1;                               // Return 1 if toggle two is on
  } else {
    return 0;                               // Return zero if reading did something awful.
  }
}

bool sometinChange() {
  if (swOneSave != digitalRead(swOne)) {
    return true;
  } else if (swTwoSave != digitalRead(swTwo)) {
  return true;
  } else if (swThreeSave != digitalRead(swThree)) {
    return true;
  } else if (swFourSave != digitalRead(swFour)) {
    return true;
  } else if (swFiveSave != digitalRead(swFive)) {
    return true;
  } else if (speedPotSave.number != potRead(speedPot)) {
    return true;
  } else if (steerPotSave.number != potRead(steerPot)) {
    return true;
  } else if (leftPotSave.number != potRead(leftPot)) {
    return true;
  } else if (rightPotSave.number != potRead(rightPot)) {
    return true;
  } else {
    return false;
  }
}

void collectData(){
  swOneSave = digitalRead(swOne);
  swTwoSave = digitalRead(swTwo);
  swThreeSave = digitalRead(swThree);
  swFourSave = digitalRead(swFour);
  swFiveSave = digitalRead(swFive);
  speedPotSave.number = potRead(speedPot);
  steerPotSave.number = potRead(steerPot);
  leftPotSave.number = potRead(leftPot);
  rightPotSave.number = potRead(rightPot);
}

void assembleData() {
  go = goToggleEncode(swOneSave, swTwoSave);
  if(!leftPotSave.number) go = 3;
  transmitData[0] = go;
  for (int i = 0; i< 2; i++){
      transmitData[i+1] = speedPotSave.bytes[i];
    }
  if (swFiveSave == 1){
      for (int i = 0; i< 2; i++){
          transmitData[i+3] = steerPotSave.bytes[i];
        }
  } else {
    transmitData[3] = 0;
    transmitData[4] = 2;
  }  
}

void lcdGo() {
  lcd.setCursor(0,0);
  if (go == 0) {
    lcd.print("STOP:");
  } else if (go == 1) {
    lcd.print("FWRD:");
  } else if (go == 2) {
    lcd.print("BACK:");
  } else {
    lcd.print("ERROR");
  }
  long speedVal;
  speedVal = speedPotSave.number;
  int speedPercent = (speedVal * 100) / 1023;
  lcd.setCursor(5,0);
  lcd.print(speedPercent);
  lcd.print("% ");
}

void lcdTD() {
  lcd.setCursor(9,0);
  lcd.print("TD:");
  float TD = rightPotSave.number / 102.3;
  lcd.setCursor(12,0);
  lcd.print(TD);
}

void lcdTurn() {
  lcd.setCursor(0,1);
  long steerVal = steerPotSave.number;
  int turnDir = steerVal / 511;
  int turnPercent;
  if (turnDir == 0) {
    turnPercent = 100 - ((steerVal * 100) / 511);
  } else {
    turnPercent = ((steerVal - 511) * 100) / 511;
  }
  if(swFiveSave == 0) {
    lcd.print("STR8:");
  } else if (swFiveSave == 1) {
    lcd.print("TRN");
    lcd.setCursor(3,1);
    if (turnDir == 0) {
      lcd.print("L:");
    } else {
      lcd.print("R:");
    }
  }
  lcd.setCursor(5,1);
  lcd.print(turnPercent);
  lcd.print("% ");
}

void lcdSW() {
  lcd.setCursor(9,1);
  lcd.print("3:");
  lcd.setCursor(11,1);
  lcd.print(swThreeSave);
  lcd.setCursor(13,1);
  lcd.print("4:");
  lcd.setCursor(15,1);
  lcd.print(swFourSave);
}

void lcdPrintAll() {
  lcdGo();
  lcdTD();
  lcdTurn();
  lcdSW();  
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

  // Setup LCD
  lcd.begin(16,2);

  //Setup digital pins
  pinMode(swOne, INPUT);
  pinMode(swTwo, INPUT);
  pinMode(swThree, INPUT);
  pinMode(swFour, INPUT);
  pinMode(swFive, INPUT);
}

void loop() {
  unsigned long time = millis();
  Serial.println(go);
  //Transmit if toggle or pot has changed position
  if (sometinChange()){
          collectData();
          assembleData();
      
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
            //lcd.clear();
            //lcd.print("Transmit Failed!"); 
            lcdPrintAll();
            Serial.print("Throttle: ");
            Serial.println(speedPotSave.number);
            Serial.print("Turn: ");
            Serial.println(steerPotSave.number);
          } else {
            wireless.read( &receiveData, sizeof(receiveData) );
               // Print data
            Serial.println("*******************************************");
            Serial.print("Got response: ");
            Serial.println(receiveData[0]);
            lcdPrintAll();
          }
  }


  delay(20); // wait a 20 milliseconds and look again. 

  
}

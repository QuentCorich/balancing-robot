/*
 * Authors: Quentin Corich, Austin Hutchison, Alec Richey, Braedon Lieberman
 * 
 * Code for Sensor Controller of balancing robot. Designed for Arduino Uno (0)
 * 
 * Detect color beneath robot: APDS-9960 Color and Gesture Sensor(1)
 * SparkFun Logic Level Converter - Bi-Directional(2)
 * Detect objects infront of robot: HC-SR04 Ultrasonic Sensor(3)
 * I2C Communication(4)
 * 
 * Sources:
 * (0) https://www.arduino.cc/en/Main/ArduinoBoardUno
 * (0) http://www.atmel.com/devices/atmega328p.aspx (controller datasheet)
 * (1) https://www.sparkfun.com/products/12787
 * (2) https://www.sparkfun.com/products/12009?gclid=CIq4i926u8gCFQutaQod5p8G6g
 * (3) https://docs.google.com/document/d/1Y-yZnNhMYy7rwhAgyL_pfa39RsB-x2qR4vP8saG73rE/edit
 * (3) http://randomnerdtutorials.com/complete-guide-for-ultrasonic-sensor-hc-sr04/
 * (4) https://www.arduino.cc/en/Reference/Wire
 * (4) https://learn.sparkfun.com/tutorials/i2c (info on I2C)
 */

 /* Libraries */
#include <Wire.h>
#include <SparkFun_APDS9960.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>
#include "printf.h"


/* - RGB sensor Wiring -
 * MUST USE 
 * 3.3V         VCC              Power
 * GND          GND              Ground
 * A4           SDA              I2C Data
 * A5           SCL              I2C Clock
 */

/* - I2C Wiring -
 * UNO:
 * A4 - SDA
 * A5 - SCL
 * 
 * MEGA 2560:
 * 20 - SDA
 * 21 - SCL
 */

/* - Wireless transeiver Wiring -
 * vcc   3.3v
 * GND   gnd
 * CE    8
 * CSN   10
 */

 
/* Definitions */
//pins
#define ambienceLED 5                      //White LED provide consistant ambience for RGB sensor
#define indicatorLED 7                        //indicator LED illuminates when error occures
#define errorLED 6                        //indicator LED illuminates when error occures
#define echo 2                             //external interrupt pin 2 (INT0) for distance sensor
#define RGB_int 3                          //external interrupt pin 3 (INT1) for RGB sensor
#define CE 8                               //Wireless pin
#define CSN 10                             //Wireless pin
#define trigger 9                          //must be 9 (OCR1A) or 10 (OCR1B)
//constants
#define control_address 5                  //I2C addressing
#define LIGHT_INT_HIGH 100000              //RGB interrupt occures if ambiant light exceeds this value
#define LIGHT_INT_LOW 32                   //RGB interrupt occures if ambiant light falls bellow this value


/* Global Variables */
SparkFun_APDS9960 apds = SparkFun_APDS9960();
uint16_t ambient_light = 0;                //measured ambient light
uint16_t red_light = 0;                    //measured temperature of red
uint16_t green_light = 0;                  //measured temperature of green
uint16_t blue_light = 0;                   //measured temperature of blue
char color = 'n';                          //value set based upon RGB readings from RGB sensor
unsigned long init_wait = 0;               //wait timer for RGB interrupt to allow rest of program to initilize
double distanceCM[5] = {0, 0, 0, 0, 0};    //store measured distances in cm
int distancePointer = 0;                   //used to iterate through distanceCM
double echoTimer = 0;                      //used to time delay from signal to echo to calc distance with
int machine_state = 0;                     //value recieved from remote control to tell control action to perform (renamed from direction_input)
int velocity_input = 0;                    //value recieved from remote control to tell robot to move
int steering_input = 512;                    //
unsigned long frame_rate = millis();       //regulates rate at which data is sent to control
unsigned long COM_rate = millis();         //regulates rate at which data is sent through COM
bool i2c_transmit = false;                     //set true when a new frame needs to be sent to control
int i2c_nack = 0;                          //error handling, 0 - successful i2c communication, 1-6 - unsuccessful
int noEchoCount = 0;                       //error handling, counts when there is no echo to a trigger
int errorState = 0;

/*  machine_state
 * 0 - no action
 * 1 - move forward
 * 2 - move backwards
 * 3 - follow
 * 8 - object/line detected
 */
 
 
 /* Wireless Declarations */
// Intialize transeiver
RF24 wireless(CE, CSN);
// Intialize transmit packet
uint8_t transmitData[1];                  // Transmit 1 byte packet
//Setup Recieve Packet
typedef union
{
 int number;
 uint8_t bytes[2];
} intOrByte;
uint8_t recieveData[5];                  // Recieve 3 byte packet
int go = 0;                              // 0=stop; 1=fwd; 2=bck
intOrByte goSpeed;
intOrByte goSteer;
// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = {0xe7e7e7e7e7LL, 0xc2c2c2c2c2LL};




void setup() {
  Serial.begin(9600);//initilize COM
  Wire.begin(); //initilize I2C
  //line_detection_initilization(); //must be initilized before anything else!!!
  wireless_communication_initilization();
  object_detection_initilization();
  error_handler_initilization();
  initilizationSuccessful();
}


void loop() {  
  //checks for input from remote control
  recieveWirelessData();
  //clear interrupt for RGB sensor
  //if (!apds.clearAmbientLightInt())
  //  color = 'e';

  if(machine_state == 3){
    int dis = getDistance();
    steering_input = 512;
    if(dis > 30) velocity_input = -1000;
    else if(dis < 25) velocity_input = 1000;
    else velocity_input = 0;
  }
  
  if(millis() > COM_rate){ //sends info through COM at 1 Hz
    printDistance();
    printColor();
    printRecievedData();
    printError();
    Serial.println("---------------------------------------");
    COM_rate = millis() + 1000;
  }
  if(millis() > frame_rate && i2c_transmit){ //sends info to control (through I2C) at 100 Hz max
    sendFrame(control_address, machine_state, velocity_input, steering_input);
    i2c_transmit = false;
    frame_rate = millis() + 10;
  }
  //Feedback from controller in event of an issue
  catchError(); //catches I2C communication error, RGB sensor error, object detection error
}

/* ----------------------------------- Line Detection ------------------------------------ */
void line_detection_initilization(){ //must be initilized first!!!
  Serial.println("------------------- APDS-9960 Setup -------------------");
  pinMode(RGB_int, INPUT);
  pinMode(ambienceLED, OUTPUT);
  digitalWrite(ambienceLED, HIGH); //turn on ambient LED
  init_wait = millis() + 2000; //Set wait time to 2 seconds
  SREG = 0x80; //global interrupts enabled
  EICRA |= 0x08; //interrupt at any falling edge of INT1 (pin 3)
  EIMSK |= 0x02; //enable interrupts for INT1
  // Initialize APDS-9960 (configure I2C and initial values)
  if(apds.init()) Serial.print(F("APDS-9960 initialization successful"));
  else{
    color = 'e';Serial.print(F("******ERROR: APDS-9960 initialization******"));
  }
  Serial.print("  ---  ");
  // Set high and low interrupt thresholds
  if ( !apds.setLightIntLowThreshold(LIGHT_INT_LOW) ){
    color = 'e';Serial.println(F("Error writing low threshold"));
  }
  if ( !apds.setLightIntHighThreshold(LIGHT_INT_HIGH) ){
    color = 'e';Serial.println(F("Error writing high threshold"));
  } 
  // Start running the APDS-9960 light sensor (no interrupts)
  //disable interrupts
  if(apds.enableLightSensor(false)) Serial.println(F("Light sensor enabled"));
  else{
    color = 'e';Serial.println(F("******ERROR: Light sensor******"));
  }
  // Read high and low interrupt thresholds
  uint16_t threshold = 0;
  if ( !apds.getLightIntLowThreshold(threshold) ) {
    color = 'e';Serial.println(F("Error reading low threshold"));
  } else {
    Serial.print(F("Low Threshold: "));Serial.println(threshold);
  }
  if ( !apds.getLightIntHighThreshold(threshold) ) {
    color = 'e';Serial.println(F("Error reading high threshold"));
  } else {
    Serial.print(F("High Threshold: "));Serial.println(threshold);
  }
  // Enable interrupts
  if (!apds.setAmbientLightIntEnable(1)){
    color = 'e';Serial.println(F("Error enabling interrupts"));
  }
  Serial.println();
  delay(500); // Wait for initialization and calibration to finish
}

ISR(INT1_vect){ //triggers when ambiant is above or bellow threshold
  if(millis() > init_wait){ //wait for the rest of initilization before detecting line
    if(machine_state == 1){ //only detect line if robot is moving forward
      color = 'd'; //display that the line has been detected
      machine_state = 8;
      i2c_transmit = true;
    }
  }
}

void printColor(){ //print data collected from sensor
  if(!apds.readAmbientLight(ambient_light) || !apds.readRedLight(red_light) ||
     !apds.readGreenLight(green_light) || !apds.readBlueLight(blue_light))
    color = 'e'; //Error in reading light values
  else {
    red_light *= .57;
    green_light *= .36;
    blue_light *= .28;
  }  
  Serial.print("Red:  ");
  Serial.print(red_light);
  Serial.print("\tGreen:  ");
  Serial.print(green_light);
  Serial.print("\tBlue:  ");
  Serial.println(blue_light);
  Serial.print("Color: ");
  Serial.println(color);
  Serial.print("Ambiant:  ");
  Serial.println(ambient_light);
  Serial.println();
}
 /* --------------------------------------------------------------------------------------- */

 /* ---------------------------------- Object Detection ----------------------------------- */
void object_detection_initilization(){
  Serial.println("------------------- Object Detection Setup -------------------");
  //initilize pins
  pinMode(trigger, OUTPUT);
  digitalWrite(trigger, LOW);
  pinMode(echo, INPUT);
  //initilize Trigger logic (16 bit Timer/Counter 1)
  TCCR1A = 0x82; //set to fast pwm, non-inverting mode
  TCCR1B = 0x1B; //set to fast pwm, clk prescaler 64
  OCR1A = 0x0003; //12us pulse width
  ICR1 = 0x03E8; // 40 ms period
  TIMSK1 = 0x02; //interrupt at OCR1A compair match
  //initilize Echo logic (external interrupt at pin 2)
  SREG = 0x80; //global interrupts enabled
  EICRA |= 0x01; //interrupt at any logical change of INT0 (pin 2)
  EIMSK |= 0x01; //enable interrupts for INT0
  Serial.println("Initilization successful");Serial.println();
}

//noEchoCount - accumulates as triggers are sent out with no echo
//controller interprets more than 8 in a row as an issue
ISR(TIMER1_COMPA_vect){noEchoCount++;}

ISR(INT0_vect){ //clocks time between rising edge and falling edge of echo, calculates distance
  if(digitalRead(echo)){//if echo HIGH begin counting, if echo LOW determine time elapsed
    echoTimer = micros();//set timer
  }else{
    echoTimer = micros() - echoTimer;//determine time elapsed (in us)
    distanceCM[distancePointer] = echoTimer / 58.0;//calculate disntance in CM
    if(distanceCM[distancePointer] > 400) distanceCM[distancePointer] = 400; //upper limit of sensor
    if(distanceCM[distancePointer] < 2) distanceCM[distancePointer] = 2; //lower limit of sensor
    else noEchoCount = 0;//resets due to having recieved an echo
    //Stop if object is at or less than 20cm away
    if(getDistance() <= 20 && machine_state == 1){
      velocity_input = 0;
      machine_state = 8;
      i2c_transmit = true;
    }
    //iterate pointer
    distancePointer++;
    if(distancePointer >= (sizeof(distanceCM)/sizeof(double))) distancePointer = 0;
  }
}

//returns the average of the previous 5 distance readings
//attempt to reduce issues due to background ultrasonic noise
double getDistance(){
  double avg = 0;
  for(int i = 0; i < (sizeof(distanceCM)/sizeof(double)); i++)
    avg += distanceCM[i];
  return avg / (sizeof(distanceCM)/sizeof(double));
}

void printDistance(){ //print distance info
  Serial.print("Distance: ");
  for(int i = 0; i < (sizeof(distanceCM)/sizeof(double)); i++){
    Serial.print(distanceCM[i]);
    Serial.print("\t");
  }
  Serial.print("Average: ");
  Serial.println(getDistance());
  Serial.println();
}
 /* --------------------------------------------------------------------------------------- */

 /* --------------------------------- I2C Communication ----------------------------------- */
void sendFrame(int address, int datum1, int datum2, int datum3){ //sends two integers
  i2c_nack = 6;
  Wire.beginTransmission(address);   // Send to device of corresponding address
  Wire.write(datum1);
  Wire.write((char*)&datum2, sizeof(datum2));//Sends velocity as a series of 2 bytes
  Wire.write((char*)&datum3, sizeof(datum3));//Sends steering as a series of 2 bytes
  i2c_nack = Wire.endTransmission();      // Stop Transmitting
  //i2c_nack: 0-success,  1-data too big, 2-error at address transmit
  //3-error at data transmit, 4-other error
}
 /* --------------------------------------------------------------------------------------- */

 /* ------------------------------- Wireless Communication -------------------------------- */

 void wireless_communication_initilization(){
  Serial.println("--------------- Wireless Communication ----------------");
  printf_begin();
 // Setup wireless
  wireless.begin();
  wireless.openWritingPipe(pipes[1]);
  wireless.openReadingPipe(1,pipes[0]);
  wireless.startListening();
  wireless.printDetails();
  Serial.println();
 }
 void recieveWirelessData(){
  // Receive Code
  if (wireless.available()){
   // Serial.println("***************************************");
    bool done = false;
    while(!done){
      done = wireless.read( &recieveData, sizeof(recieveData) );
    }
  
  //Decode wireless data
  go = recieveData[0];
  for (int i = 0; i < 2; i++){
    goSpeed.bytes[i] = recieveData[i+1];
  }
  for (int i = 0; i < 2; i++){
    goSteer.bytes[i] = recieveData[i+3];
  }
  if(go != 1 || machine_state != 8)
    machine_state = go;
  velocity_input = goSpeed.number;
  steering_input = goSteer.number;
  
  //Copy rx data packet into transmit data packet
  transmitData[0] = recieveData[0];
  
  // Transmit Code  (transmit back to indicate packet recieved)
  wireless.stopListening();
  wireless.write( &transmitData, sizeof(transmitData) );
  wireless.startListening();
  i2c_transmit = true;
  }
 }
 void printRecievedData(){
    Serial.print("velocity_input: ");
    Serial.print(velocity_input);
    Serial.print("\t");
    Serial.print("machine_state: ");
    Serial.println(machine_state);
    Serial.print("\t");
    Serial.print("steering_input: ");
    Serial.println(steering_input);
    Serial.println();
 }
/* --------------------------------------------------------------------------------------- */

/* ----------------------------------- Error Handler ------------------------------------- */

void error_handler_initilization(){
  pinMode(indicatorLED, OUTPUT);
  digitalWrite(indicatorLED, LOW);
  pinMode(errorLED, OUTPUT);
  digitalWrite(errorLED, LOW);
}

void initilizationSuccessful(){
  digitalWrite(indicatorLED, HIGH);
  delay(1000);
  digitalWrite(indicatorLED, LOW);
}

void catchError(){
  if(i2c_nack != 0){
    errorState = 1;
    digitalWrite(errorLED, HIGH);
  }
  //else if(color == 'e'){
  //  errorState = 2;
  //  digitalWrite(errorLED, HIGH);
  //}
  else if(noEchoCount > 8){
    errorState = 3;
    digitalWrite(errorLED, HIGH);
  }
  else{
    errorState = 0;
    digitalWrite(errorLED, LOW);
  }
}

void printError(){
  switch (errorState){
    case 0:
    
    break;
    
    case 1:
      Serial.println("******ERROR: I2C Communication******");
    break;
    
    case 2:
      Serial.println("******ERROR: RGB sensor******");
    break;
    
    case 3:
      Serial.println("******ERROR: Object Detection******");
    break;
  }
}
/* --------------------------------------------------------------------------------------- */

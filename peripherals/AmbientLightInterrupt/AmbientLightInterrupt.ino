/****************************************************************
AmbientLightInterrupt.ino
APDS-9960 RGB and Gesture Sensor
Shawn Hymel @ SparkFun Electronics
October 24, 2014
https://github.com/sparkfun/APDS-9960_RGB_and_Gesture_Sensor

Tests the ambient light interrupt abilities of the APDS-9960.
Configures the APDS-9960 over I2C and waits for an external
interrupt based on high or low light conditions. Try covering
the sensor with your hand or bringing the sensor close to a
bright light source. You might need to adjust the LIGHT_INT_HIGH
and LIGHT_INT_LOW values to get the interrupt to work correctly.

Hardware Connections:

IMPORTANT: The APDS-9960 can only accept 3.3V!
 
 Arduino Pin  APDS-9960 Board  Function
 
 3.3V         VCC              Power
 GND          GND              Ground
 A4           SDA              I2C Data
 A5           SCL              I2C Clock
 2            INT              Interrupt
 13           -                LED

Resources:
Include Wire.h and SparkFun_APDS-9960.h

Development environment specifics:
Written in Arduino 1.0.5
Tested with SparkFun Arduino Pro Mini 3.3V

This code is beerware; if you see me (or any other SparkFun 
employee) at the local, and you've found our code helpful, please
buy us a round!

Distributed as-is; no warranty is given.
****************************************************************/

#include <Wire.h>
#include <SparkFun_APDS9960.h>

// Pins
#define APDS9960_INT 3  // Needs to be an interrupt pin

// Constants
#define LIGHT_INT_HIGH  10000 // High light level for interrupt
#define LIGHT_INT_LOW   1000   // Low light level for interrupt

// Global variables
SparkFun_APDS9960 apds = SparkFun_APDS9960();
uint16_t ambient_light = 0;
uint16_t red_light = 0;
uint16_t green_light = 0;
uint16_t blue_light = 0;
int isr_flag = 0;
uint16_t threshold = 0;
char color = 'n';
int machine_state = 0;
unsigned long init_wait = 0;
unsigned long intCount = 0;

void setup() {
  pinMode(APDS9960_INT, INPUT);
  
  // Initialize Serial port
  Serial.begin(9600);
  Serial.println();
  Serial.println(F("-------------------------------------"));
  Serial.println(F("SparkFun APDS-9960 - Light Interrupts"));
  Serial.println(F("-------------------------------------"));
  
  // Initialize interrupt service routine
  //attachInterrupt(1, interruptRoutine, FALLING);
  init_wait = millis() + 2000;
  SREG = 0x80; //global interrupts enabled
  EICRA |= 0x08; //interrupt at any falling edge of INT1 (pin 3)
  EIMSK |= 0x02; //enable interrupts for INT1
  // Initialize APDS-9960 (configure I2C and initial values)
  if ( apds.init() ) {
    Serial.println(F("APDS-9960 initialization complete"));
  } else {
    Serial.println(F("Something went wrong during APDS-9960 init!"));
  }
  
  // Set high and low interrupt thresholds
  if ( !apds.setLightIntLowThreshold(LIGHT_INT_LOW) ) {
    Serial.println(F("Error writing low threshold"));
  }
  if ( !apds.setLightIntHighThreshold(LIGHT_INT_HIGH) ) {
    Serial.println(F("Error writing high threshold"));
  }
  
  // Start running the APDS-9960 light sensor (no interrupts)
  if ( apds.enableLightSensor(false) ) {
    Serial.println(F("Light sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during light sensor init!"));
  }
  
  // Read high and low interrupt thresholds
  if ( !apds.getLightIntLowThreshold(threshold) ) {
    Serial.println(F("Error reading low threshold"));
  } else {
    Serial.print(F("Low Threshold: "));
    Serial.println(threshold);
  }
  if ( !apds.getLightIntHighThreshold(threshold) ) {
    Serial.println(F("Error reading high threshold"));
  } else {
    Serial.print(F("High Threshold: "));
    Serial.println(threshold);
  }
  
  // Enable interrupts
  if ( !apds.setAmbientLightIntEnable(1) ) {
    Serial.println(F("Error enabling interrupts"));
  }else Serial.println("interrupt enabled");
  
  // Wait for initialization and calibration to finish
  delay(500);
}

void loop() {
  if (  !apds.readAmbientLight(ambient_light) ||
        !apds.readRedLight(red_light) ||
        !apds.readGreenLight(green_light) ||
        !apds.readBlueLight(blue_light) ) {
    Serial.println("Error reading light values");
  } else {

    //red_light = (red_light * 100) / (ambient_light * .85);  //Normalize values based on ambience
    //green_light = (green_light * 100) / (ambient_light * .95);//ratios based on normalized responsivity graph
    //blue_light = (blue_light * 214) / (ambient_light * .70);
    
    red_light *= .57;
    green_light *= .36;
    blue_light *= .28;
    
    Serial.print("Ambient: ");
    Serial.print(ambient_light);
    Serial.print(" R: ");
    Serial.print(red_light);
    Serial.print(" G: ");
    Serial.print(green_light);
    Serial.print(" B: ");
    Serial.print(blue_light);
    Serial.print("  Color: ");
    Serial.print(color);
    Serial.print("  ");
    Serial.print("machine_state: ");
    Serial.print(machine_state);
    Serial.print("  Count:  ");
    Serial.println(intCount);
    if (!apds.clearAmbientLightInt())
      Serial.println("******ERROR: RGB Interrupt******");
  }
}

ISR(INT1_vect){ 
  intCount++;
  if(millis() > init_wait){
  if (!apds.clearAmbientLightInt())
      Serial.println("******ERROR: RGB Interrupt******");
  }else Serial.println("initilizing...");
}
void detectRed(){ //changes variable color when red is detected
  if(red_light > 100 && red_light > green_light && red_light > blue_light)
    color = 'r';
  else color = 'n';
}

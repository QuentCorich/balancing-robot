/*
 * Authors: Quentin Corich, Austin Hutchison
 * 
 * Detect objects infront of the balancing robot using 
 * HC-SR04 ultrasonic sensor
 * 
 * sources:
 * http://www.atmel.com/devices/atmega328p.aspx
 * https://docs.google.com/document/d/1Y-yZnNhMYy7rwhAgyL_pfa39RsB-x2qR4vP8saG73rE/edit
 * http://randomnerdtutorials.com/complete-guide-for-ultrasonic-sensor-hc-sr04/
 */

//define pins
#define trigger 11
#define echo 18

//global variables
double distanceCM = 0;
double echoTimer = 0;

void setup() {

  Serial.begin(9600);
  //pin initilization
  pinMode(trigger, OUTPUT);
  digitalWrite(trigger, LOW);
  pinMode(echo, INPUT);
  digitalWrite(echo, LOW);
  
  //initilize Trigger logic (16 bit Timer/Counter 1)
  TCCR1A = 0x82; //set to phase correct pwm mode, non-inverted
  TCCR2B = 0x12; //clk prescaler 8
  ICR1 = 40000; // 40ms period
  OCR1A = 0x000A; //10us pulse width
  TIMSK1 = 0x00; //no interrupt
  
  //initilize Echo logic (external interrupt at pin 18)
  SREG = 0x80; //global interrupts enabled
  EICRA = 0x40; //interrupt at any logical change of INT3 (pin 18)
  EIMSK = 0x08; //enable interrupts for INT3
    
}

void loop() {
  
  if(distanceCM > 400) distanceCM = 400; //upper limit of sensor
  if(distanceCM < 2) distanceCM = 2; //lower limit of sensor

  Serial.print("Distance:   ");
  Serial.println(distanceCM);
  if(distanceCM <= 10){
    //warn object is near
  }
}

/*
 * interrupts at both rising edges and falling edges at pin 2.
 * If statement determines if interrupt was due to a rising edge or falling edge.
 * micros() is used to time the length of the pulse in microseconds. When a pulse is
 * sucessfully timed a formula is used to determine distance
 */
ISR(INT3_vect){
  if(digitalRead(echo)){
    echoTimer = micros();
  }else{
    echoTimer = micros() - echoTimer;
    distanceCM = echoTimer / 58.0;
  }
}




/*
 * Authors: Alec Richey, Braeden Lieberman, Quentin Corich, Austin Hutchison
 * 
 * Code for Control Portion of Balancing robot. Designed for Arduino MEGA 2560(0)
 * 
 * Pololu Dual MC33926 Motor Driver Shield for Arduino(1)
 * 50:1 Metal Gearmotor 37Dx54L mm with 64 CPR Encoder(2)
 * SparkFun Gyro Breakout - LPY503AL(3)
 * I2C Communication(4)
 * 
 * Sources:
 * (0) https://www.arduino.cc/en/Main/ArduinoBoardMega2560
 * (0) http://www.atmel.com/devices/atmega2560.aspx (controller datasheet)
 * (1) https://www.pololu.com/product/2503/resources
 * (1) https://github.com/pololu/dual-mc33926-motor-shield (Motor Shield Library)
 * (2) https://www.pololu.com/product/1444/specs
 * (2) http://www.pjrc.com/teensy/td_libs_Encoder.html (Encoder Library)
 * (3) https://www.sparkfun.com/products/11341
 * (4) https://www.arduino.cc/en/Reference/Wire
 */


 /* Libraries */
#include "DualMC33926MotorShield.h"
#include <Encoder.h>
#include <Wire.h>


/* - Motor&Encoder Wiring -
 * Wire Color    Motor1      Motor2      Function
 *  red/black     M1A/M2B     M2A/M2B     Motor Power
 *  green         gnd         gnd         Ground
 *  blue          5V          5V          Source
 *  yellow        2           19          Encoder I/O A
 *  white         3           18          Encodor I/O B
 */

 /* - GyroScope -
  * zeroValue = 1230.0 (defined)
  * radian = (pi/180) * degree
  * resolution: 8.3 mV / (degree / s)
  */


 /* Definitions */
//pins
#define encA1 2                            //Hall sensor A output motor 1
#define encB1 3                            //Hall sensor B output motor 1
#define encA2 18                           //Hall snesor A output motor 2
#define encB2 19                           //Hall sensor B output motor 2
#define enc1VCC 14                         //Power for encoder 1
#define enc1GND 15                         //Ground for encoder 1
#define enc2VCC 16                         //Power for encoder 2
#define enc2GND 17                         //Ground for encoder 2
#define MD_reset 4                         //Motor Drive reset pin
#define m1Dir 7                            //motor 1 direction pin
#define m2Dir 8                            //motor 2 direction pin
#define m1PWM 9                            //motor 1 pwm
#define m2PWM 10                           //motor 2 pwm
#define status_flag 12                     //motor drive status flag
#define HP 22                              //gyro high pass filter reset
#define PD 24                              //gyro power down
#define ST 26                              //gyro self Test
#define x_axis 8                           //gyro output
#define z_axis 9                           //gyro output
#define gyro_sensitivity 8.3               //33.3 for 4x amp, 8.3 for 1x amp
//constants
#define address 5                          //I2C addressing
#define zeroValue 1230.0                   //gyro constant (in mV)
#define countsPerRad 509                   //about equal to 3200 counts per revolution divided by 2pi rad/rev/
#define ki1 9.76                           //
#define kp1 0.5474                         //
#define ki2 9.76                           //ki and kp constants from modeling
#define kp2 0.5474                         //
#define scale1 21.65                       //relation between motor steps and radians
#define scale2 21.65                       //
#define PIwindup .5                        //max windup of velocity control

//PID Controler: Xdot(s) = K * (s+a) /( (s+b)(s+c) )
#define K 5000                             //balance PID amplitude
#define a 38                               //balance PID hole value
#define b -17                              //balance PID pole value
#define c 7.25                             //balance PID pole value
//Control values for phiRef
//#define Kx -0.015                          //proportional control
//#define Kxi -0.0021                        //integral control
#define Kx -0.020                            //proportional control
#define Kxi 0                                //integral control

 /* Global Variables */
float x_input = 0;                         //x-axis angular velocity of gyro (rad/s)
float z_input = 0;                         //z-axis angular velocity of gyro (rad/s)
float x_offset = .010;                     //corrects for drift, setOffset() sets this at initilization
float z_offset = .010;                     //corrects for drift, setOffset() sets this at initilization

float setVelocity = 0.0;                   //reference value for velocity control
int machine_state = 0;                     //controls operation of robot
int turnDir = 0;                           //
float turnPercent = 0.0;                   //

float deltaTime = 0.0;                     //time between each angular acceleration sample
unsigned long timerNew = 0.0;              //time of current sample
unsigned long timerOld = 0;                //time of previous sample
signed long countNew1 = 0;                 //encoder 1 value of current sample
signed long countNew2 = 0;                 //encoder 1 value of previous sample
long countOld1 = 0;                        //encoder 2 value of current sample
long countOld2 = 0;                        //encoder 2 value of previous sample
long prev_count1 = 0;                      //retains previous sample time for printing to com
long prev_count2 = 0;                      //use prev_count instead of countOld
float velocity1 = 0.0;                     //calculated velocity of motor 1
float velocity2 = 0.0;                     //calculated velocity of motor 2

float P1 = 0;                              //proportional control amplitude
float P2 = 0;                              //
float I1 = 0;                              //integral control amplitude
float I2 = 0;                              //
float error1 = 0.0;                        //velocity errors
float error2 = 0.0;                        //
float integral1 = 0.0;                     //integrates error in motor 1
float integral2 = 0.0;                     //integrates error in motor 2
float PWM1 = 0.0;                          //Value fed to motor driver to set PWM
float PWM2 = 0.0;                          //

float phi = 0.0;                           //
float phiIntegral = 0;                     //
float phiRef = 0;                          //angular velocity reference value for balancing
float phiError = 0;                        //angular velocity error in balancing
float v = 0;                               //partial term for balance controller
float v_old = 0;                           //historisis for partial term
float XdotRef = 0;                         //output from PID controller
float XdotRef_old = 0;                     //historisis for PID controller

unsigned long COM_rate = millis();         //rate of data being printed to com


 /* Object Declaration */
DualMC33926MotorShield md;
Encoder rotary1(encA1,encB1);
Encoder rotary2(encA2,encB2);

void setup() {

  Serial.begin(9600);
  Wire.begin(address); //initilize I2C
  Wire.onReceive(receiveFrame);
  md.init(); //initilize motor driver
  gyroscope_initilization();
  control_initilization();
  countOld1 = countNew1 = rotary1.read();//makes velocity control happy
}

void loop() {
  
  if(millis() > COM_rate){ //prints to COM at rate of 1 Hz
    printRecieved();
    Serial.println("---------------------------------------");
    COM_rate = millis() + 1000;
}
}
 
 /* --------------------------------------- Control ---------------------------------------- */

 void control_initilization(){
  //Initilize encoders
  pinMode(enc1VCC, OUTPUT);
  digitalWrite(enc1VCC, HIGH);
  pinMode(enc1GND, OUTPUT);
  digitalWrite(enc1GND, LOW);
  pinMode(enc2VCC, OUTPUT);
  digitalWrite(enc2VCC, HIGH);
  pinMode(enc2GND, OUTPUT);
  digitalWrite(enc2GND, LOW);
  //Initilize 16-bit Counter/Timer
  TCCR3A = 0x00;    //Set to CTC mode - resets counter register (TCNT1) upon compare match
  TCCR3B = 0x0C;    // clock prescaler: /256
  TIMSK3 = 0x02;    // interrupt at OCR1A compare match (TOP value)
  OCR3A = 0x004E;   //set compare match value
  SREG = 0x80;      //global interrupt enable
 }
 
 /*  
  * TIMER1_COMPA_vect interrupts once every 1.25 ms (TOP value * prescaler / clk = 0x4E * 256 / 16000000)
  * Placing control inside of an interrupt makes sampling rate for the control constant
  * and eliminates the possibility of an interrupt taking place during calculations,
  * thus stabilizing the control.
  */
 ISR(TIMER3_COMPA_vect){
  timerNew = micros();//timerNew in microseconds, increments throughout running of code
  deltaTime = (timerNew - timerOld)/1000000.0;//Sample Time (in seconds)
  
  /* Balance Control */
  calcAngularVelocity();
  countNew1 = rotary1.read();   //set countNew to however many counts the encoder has done
  countNew2 = rotary2.read();
  if (deltaTime > 0) {        //only let velocity calculated if deltaTime is not 0, don't want to divide by 0
     velocity1 = ((float)(countNew1 - countOld1)/countsPerRad)/deltaTime;   //get velocity in radians per second
     velocity2 = ((float)(countNew2 - countOld2)/countsPerRad)/deltaTime;
  }
  else {
     velocity1 = 0.0;          //just make sure velocity is 0 for initial conditions
     velocity2 = 0.0;
  }
  phi = setVelocity - velocity1; //allows robot to move forward and backwards
  if(phi >= 10) phi = 10;  //limits how much phiRef can be changed at once
  if(phi <= -10) phi = -10;//this helps keep balance control stable 
  phiIntegral += (phi * deltaTime);
  if(machine_state == 0) phiIntegral = 0;//dump residual when stopped
  phiRef = -(Kx * phi + Kxi * phiIntegral);
  phiError = phiRef - x_input;
  
  v = (v_old - K * deltaTime * phiError) / (b * deltaTime + 1); //Error
  XdotRef = 1 * (XdotRef_old * deltaTime + (1 + a * deltaTime ) * v - v_old) / (1 - deltaTime * c); //PID balance controller
    
  XdotRef_old = XdotRef;
  v_old = v;

  /* Velocity Control */
  
  prev_count1 = countOld1;
  countOld1 = countNew1;        //set counter and timer old values to the new values for reference
  prev_count2 = countOld2;
  countOld2 = countNew2;
  timerOld = timerNew;
  
  //Error for PI motor control
  if(turnDir) error1 = XdotRef - velocity1 - (.02 * turnPercent);
  else error1 = XdotRef - velocity1 + (.02 * turnPercent);
  if(turnDir) error2 = XdotRef - velocity2 + (.02 * turnPercent);
  else error2 = XdotRef - velocity2 - (.02 * turnPercent);
  
  //PI motor control with windup limit
  integral1 = integral1 + error1 * deltaTime;
  if(integral1 > PIwindup) integral1 = PIwindup;
  if(integral1 < (-1 * PIwindup)) integral1 = -1 * PIwindup;
  integral2 = integral2 + error2 * deltaTime;
  if(integral2 > PIwindup) integral2 = PIwindup;
  if(integral2 < (-1 * PIwindup)) integral2 = -1 * PIwindup;
  
  I1 = integral1*ki1;
  I2 = integral2*ki2;
  P1 = error1*kp1;
  P2 = error2*kp2;
  
  PWM1 = (I1 + P1)*scale1;
  if(PWM1 > 255) PWM1 = 255;
  if(PWM1 < -255) PWM1 = -255;
  PWM2 = (I2 + P2)*scale2;
  if(PWM2 > 255) PWM2 = 255;
  if(PWM2 < -255) PWM2 = -255;
  
  motor(1, PWM1);
  motor(2, PWM2);
}

void printVelocities(){ //prints measured velocity from both motors
  Serial.print(countOld1);
  Serial.print("\t");
  Serial.print(prev_count1);
  Serial.print("\t");
  Serial.print(countOld2);
  Serial.print("\t");
  Serial.print(prev_count2);
  Serial.print("\t");
  Serial.println(deltaTime);
  Serial.print("setVelocity = ");
  Serial.println(setVelocity);
  Serial.print("machine_state = ");
  Serial.println(machine_state);

  Serial.print("error1 = ");
  Serial.print(error1);            
  Serial.print("\t error2 = ");
  Serial.print(error2);
  Serial.println();

  Serial.print("integral1 = ");
  Serial.print(integral1);            
  Serial.print("\t integral2 = ");
  Serial.print(integral2);
  Serial.println();

  Serial.print("PWM1 = ");
  Serial.print(PWM1);            
  Serial.print("\t PWM2 = ");
  Serial.print(PWM2);
  Serial.println();
  Serial.print("XdotRef: ");
  Serial.println(XdotRef);
  Serial.println("--------------------------------------");
}

void motor (int m, int pwm) {  //input m = 1 for m1, m = 2 for m2, pwm between -255 and 255
  if(m == 1){
    if(pwm > 0 && pwm < 255){               //if pwm > 0, turn m1 forward direction at some speed
      digitalWrite(m1Dir,HIGH);
      analogWrite(m1PWM,pwm);
    }
    else if(pwm < 0 && pwm > -255){          //if pwm < 0, turn m1 reverse direction at some speed
      digitalWrite(m1Dir,LOW);
      analogWrite(m1PWM,-pwm);
    }
    else if (pwm >= 255){       //if pwm greater than maximum amount, just make it max
      digitalWrite(m1Dir,HIGH);
      analogWrite(m1PWM,255);
    }
    else if (pwm <= -255){      //if pwm less than minimum amount, just make it min
      digitalWrite(m1Dir,LOW);
      analogWrite(m1PWM,255);
    }
    else{
      analogWrite(m1PWM,0);    //if pwm = 0, turn m1 off
    }
  }                            //end motor m1
  if(m == 2){
    if(pwm < 0 && pwm > -255){               //if pwm > 0, turn m2 forward direction at some speed
      digitalWrite(m2Dir,HIGH);
      analogWrite(m2PWM,-pwm);
    }
    else if(pwm > 0 && pwm < 255){         //if pwm < 0, turn m2 reverse direction at some speed
      digitalWrite(m2Dir,LOW);
      analogWrite(m2PWM,pwm);
    }
    else if(pwm >= 255){       //if pwm greater than maximum amount, just make it max
      digitalWrite(m2Dir,LOW);
      analogWrite(m2PWM,255);
    }
    else if(pwm <= -255){      //if pwm less than minimum amount, just make it min
      digitalWrite(m2Dir,HIGH);
      analogWrite(m2PWM,255);
    }
    else{                     //if pwm = 0, turn m2 off
      analogWrite(m2PWM,0);
    }//end motor m2
  }
}

 /* --------------------------------------------------------------------------------------- */



 /* ------------------------------------- Gyroscope --------------------------------------- */

void gyroscope_initilization(){
  //pin initilization
  pinMode(x_axis, INPUT);
  pinMode(z_axis, INPUT);
  pinMode(HP, OUTPUT);
  pinMode(PD, OUTPUT);
  pinMode(ST, OUTPUT);
  digitalWrite(HP, LOW); //
  digitalWrite(PD, LOW); //LOW - normal mode
  digitalWrite(ST, LOW); //
  //reset filter for better accuracy
  highPassFilterReset();
}
//sends a 500us pulse to HP pin to generate a high pass filter reset
void highPassFilterReset(){
  digitalWrite(HP, HIGH);
  delayMicroseconds(500);
  digitalWrite(HP, LOW);
  Serial.println("High Pass Filter Reset");
}

void calcAngularVelocity(){
  //read x and z values with A/D converter
  x_input = analogRead(x_axis);
  z_input = analogRead(z_axis);
  
  x_input *= 4.9; //mV
  z_input *= 4.9; //mV
  x_input -= zeroValue;
  z_input -= zeroValue;

  x_input = (M_PI/180) * (x_input / gyro_sensitivity); // rad/sec
  z_input = (M_PI/180) * (z_input / gyro_sensitivity); // rad/sec

  x_input -= x_offset;
  z_input -= z_offset;
}

void printGyro(){ //print x and z values
  Serial.print(x_input, 6);
  Serial.print("\t");
  Serial.print(z_input, 6);
  Serial.print("\t");
}

 
 /* --------------------------------------------------------------------------------------- */


 
 /* --------------------------------- I2C Communication ----------------------------------- */

void receiveFrame(int howMany) {
  int data[howMany];
  int i = 0;
  while (Wire.available()){//loops until all bytes are read
     data[i] = Wire.read();
     i++;
  }
  //parse data from i2c
  machine_state = data[0];
  int newVelocity;
  *((char*)(&newVelocity) + 1) = data[2];//reconstruct 2 byte velocity
  *((char*)(&newVelocity) + 0) = data[1];//
  setVelocity = newVelocity / 100.0;
  long newSteering;
  *((char*)(&newSteering) + 3) = 0;//
  *((char*)(&newSteering) + 2) = 0;//
  *((char*)(&newSteering) + 1) = data[4];//reconstruct 2 byte turning value
  *((char*)(&newSteering) + 0) = data[3];//
  //calculate turning
  turnDir = newSteering / 511;
  if (turnDir == 0) {
    turnPercent = 100 - ((newSteering * 100) / 511);
  }else {
    turnPercent = ((newSteering - 511) * 100) / 511;
  }
  //Control states
  switch(machine_state){
    case 0: //Stop
      setVelocity *= 0;
    break;
    
    case 1: //move forward
      setVelocity *= -1;
    break;
    
    case 2:
      setVelocity *= 1;
    break;

    case 3:
    break;
    
    case 8:
      setVelocity *= 0;
      machine_state = 0;
    break;
    
    default:
      setVelocity = 0;
      turnPercent = 0;
    break;
  }
}

void printRecieved(){
  Serial.print("machine_state: ");
  Serial.print(machine_state);
  Serial.print("\t");
  Serial.print("setVelocity: ");
  Serial.print(setVelocity);
  Serial.print("\t");
  Serial.print("turnPercent: ");
  Serial.println(turnPercent);
}
/* --------------------------------------------------------------------------------------- */


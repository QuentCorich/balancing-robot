/*
 * Authors: Quentin Corich, Austin Hutchison
 * 
 * Data collection for deriving transfer function
 * 
 * 
 * 
 * sources:
 * https://www.sparkfun.com/products/11341
 */

#include "DualMC33926MotorShield.h"
#include <Encoder.h>
DualMC33926MotorShield md;

//define pins
#define x_axis A4
#define z_axis A5
#define HP 11 //High pass filter reset
#define PD 12 //Power down
#define ST 13 //Self Test
#define zeroValue 1230.0 //mV

#define m1Dir 7
#define m1PWM 9
#define m2Dir 8
#define m2PWM 10
#define encA1 2
#define encB1 3
#define encA2 4
#define encB2 5

//global variables
double x_input = 0;
double z_input = 0;
double x_offset = -.03;
double z_offset = -.03;

unsigned long timerOld;  //variables initialized to 0 for initial conditions
unsigned long timerNew;
unsigned long count1Old;
unsigned long count1New;
unsigned long count2Old;
unsigned long count2New;
float deltaTime;
float velocity1;
float velocity2;
int countsPerRad = 509;      //about 509 counts per 1 radian of revolution

Encoder rotary1(2,3);
Encoder rotary2(4,5);

void setup() {

  Serial.begin(9600);
  md.init();
  gyroscopeInitilization();
  setOffset();
}

void loop() {

  motorEncoderReset();
  doTest(2);
  motor(1,0);   // Send off command to motor 1
  motor(2,0);  //send off command to motor 2
  delay(2500);  //delay 2.5 seconds, really get motor to stop
  
}

void gyroscopeInitilization(){
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

void setOffset(){
  x_input = analogRead(x_axis);
  z_input = analogRead(z_axis);
  
  x_input *= 4.9; //mV
  z_input *= 4.9; //mV

  x_input -= zeroValue;
  z_input -= zeroValue;

  x_input = (M_PI/180) * (x_input / 8.3); // rad/sec
  z_input = (M_PI/180) * (z_input / 8.3); // rad/sec

  x_offset = x_input;
  z_offset = z_input;
}

void getAngularVelocity(){
  //read x and z values with A/D converter
  x_input = analogRead(x_axis);
  z_input = analogRead(z_axis);
  
  x_input *= 4.9; //mV
  z_input *= 4.9; //mV

  x_input -= zeroValue;
  z_input -= zeroValue;

  x_input = (M_PI/180) * (x_input / 8.3); // rad/sec
  z_input = (M_PI/180) * (z_input / 8.3); // rad/sec

  x_input -= x_offset;
  z_input -= z_offset;
}

void printValues(){
  //print x and z values
  Serial.print("x:  ");
  Serial.print(x_input, 6);
  Serial.print("    ");
  Serial.print("z:  ");
  Serial.print(z_input, 6);
  Serial.print("    ");
  Serial.print("Motor 1 v:  ");  //used to see what velocity is
  Serial.print(velocity1, 6);
  Serial.print("    ");
  Serial.print("Motor 2 v:  ");  //used to see what velocity is
  Serial.print(velocity2, 6);
  Serial.println();
}

void motorEncoderReset(){
  rotary1.write(0);             //reset encoder counts
  rotary2.write(0);
  timerOld = 0;  //variables initialized to 0 for initial conditions
  timerNew = 0;
  count1Old = 0;
  count1New = 0;
  count2Old = 0;
  count2New = 0;
  deltaTime = 0.0;
  velocity1 = 0.0;
  velocity2 = 0.0;
}

void doTest(double rotations){
  while(count1Old <= 3200 * rotations){     //turn wheel about 2 times, then reset everything
    unsigned long timer = millis();   //create timer and reference for 1ms sampling time
    unsigned long timerRef = timer;
    
    timerNew = millis();            //set timerNew to whatever time the program is at
    count1New = rotary1.read();       //set countNew to however many counts the encoder has seen
    count2New = rotary2.read();
    deltaTime = (timerNew - timerOld)/1000.0;   //put deltaTime into seconds
    
    if(timer < 1000) {        //step response.  Keep motor off until 1 second has passed
      motor(1,0);
      motor(2,0);
    }
     else {
      motor(1,150);
      motor(2,150);
    }

    if (deltaTime > 0){    //don't let velocity be calculated with a deltaTime = 0, don't want to divide by 0
       velocity1 = ((float)(count1New - count1Old)/countsPerRad)/deltaTime;
       velocity2 = ((float)(count2New - count2Old)/countsPerRad)/deltaTime;
    }
    else {
       velocity1 = 0.0;
       velocity2 = 0.0;
    }

    getAngularVelocity(); //take measurements from gyroscope
    printValues(); //print all values in single line on com

    count1Old = count1New;    //set "old" variables to what new ones equaled
    count2Old = count2New;
    timerOld = timerNew;
    delay(timerRef + 1 - timer);  //sample at about 1ms
    
  }
}

void motor (int m, int pwm) {  //input m = 1 for m1, m = 2 for m2, pwm between -255 and 255
  if(m == 1){
    if(pwm > 0){               //if pwm > 0, turn m1 forward direction at some speed
      digitalWrite(m1Dir,HIGH);
      analogWrite(m1PWM,pwm);
    }
    else if(pwm < 0){          //if pwm < 0, turn m1 reverse direction at some speed
      digitalWrite(m1Dir,LOW);
      analogWrite(m1PWM,pwm);
    }
    else if (pwm > 255){       //if pwm greater than maximum amount, just make it max
      digitalWrite(m1Dir,HIGH);
      analogWrite(m1PWM,255);
    }
    else if (pwm < -255){      //if pwm less than minimum amount, just make it min
      digitalWrite(m1Dir,LOW);
      analogWrite(m1PWM,-255);
    }
    else{
      analogWrite(m1PWM,0);    //if pwm = 0, turn m1 off
    }
  }                            //end motor m1
  if(m == 2){
    if(pwm > 0){               //if pwm > 0, turn m2 forward direction at some speed
      digitalWrite(m2Dir,HIGH);
      analogWrite(m2PWM,pwm);
    }
    else if(pwm < 0){         //if pwm < 0, turn m2 reverse direction at some speed
      digitalWrite(m2Dir,LOW);
      analogWrite(m2PWM,-pwm);
    }
    else if(pwm > 255){       //if pwm greater than maximum amount, just make it max
      digitalWrite(m2Dir,HIGH);
      analogWrite(m2PWM,255);
    }
    else if(pwm < -255){      //if pwm less than minimum amount, just make it min
      digitalWrite(m2Dir,LOW);
      analogWrite(m2PWM,-255);
    }
    else{                     //if pwm = 0, turn m2 off
      analogWrite(m2PWM,0);
    }//end motor m2
  }
}


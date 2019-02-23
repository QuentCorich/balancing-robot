//Alec Richey and Braeden Lieberman

#include "DualMC33926MotorShield.h"
#include <Encoder.h>
DualMC33926MotorShield md;

#define m1Dir 7
#define m1PWM 9
#define m2Dir 8
#define m2PWM 10
#define encA1 2
#define encB1 3
#define encA2 18
#define encB2 19

Encoder rotary1(encA1,encB1);
Encoder rotary2(encA2,encB2);

float ki1 = 9.76;            //ki and kp just constants from our modeling
float kp1 = 0.5474;
float ki2 = 6.42;
float kp2 = 0.185;
float setVelocity = 4.0;    //looking for 4 radians per second (about what our motor rotated at when we were modeling our motor)
float scale1 = 21.65;        //scale = 255/(11.87 rad/s)    looked at our angular velocity at full speed to calculate the scale
float scale2 = 22.58;
float error1 = 0.0;          //start error and other variables at 0 for initial conditions, will get updated later
float error2 = 0.0;
float integral1 = 0.0;
float integral2 = 0.0;
float I1 = 0.0;
float I2 = 0.0;
float P1 = 0.0;
float P2 = 0.0;
float PWM1 = 0.0;
float PWM2 = 0.0;
float velocity1 = 0.0;
float velocity2 = 0.0;
float deltaTime = 0.0;

unsigned long timerOld = 0;
unsigned long countOld1 = 0;
unsigned long countOld2 = 0;
unsigned long timerNew = 0;
unsigned long countNew1 = 0;
unsigned long countNew2 = 0;

int countsPerRad = 509;     //about equal to 3200 counts per revolution divided by 2pi rad/rev


void setup() {
  Serial.begin(9600);
  md.init();  
}

void loop() {
  timerNew = millis();        //timerNew in milliseconds, increments throughout running of code
  countNew1 = rotary1.read();   //set countNew to however many counts the encoder has done
  countNew2 = rotary2.read();
  deltaTime = (timerNew - timerOld)/1000.0;   //convert time in ms to seconds
  if (deltaTime > 0) {        //only let velocity calculated if deltaTime is not 0, don't want to divide by 0
     velocity1 = ((float)(countNew1 - countOld1)/countsPerRad)/deltaTime;   //get velocity in radians per second
     velocity2 = ((float)(countNew2 - countOld2)/countsPerRad)/deltaTime;
  }
  else {
     velocity1 = 0.0;          //just make sure velocity is 0 for initial conditions
     velocity2 = 0.0;
  }

  Serial.print("Velocity1 = ");       //using this to debug
  Serial.print(velocity1);            
  Serial.print("          Velocity2 = ");
  Serial.print(velocity2);
  Serial.println();

  countOld1 = countNew1;        //set counter and timer old values to the new values for reference
  countOld2 = countNew2;
  timerOld = timerNew;
  
  error1 = setVelocity - velocity1;    //just formulas from the PID control article
  error2 = setVelocity - velocity2;
  integral1 = integral1 + error1*deltaTime;
  integral2 = integral2 + error2*deltaTime;
  I1 = integral1*ki1;
  I2 = integral2*ki2;
  P1 = error1*kp1;
  P2 = error2*kp2;
  PWM1 = (I1 + P1)*scale1;
  PWM2 = (I2 + P2)*scale2;
  
  motor(1,PWM1);           //run motor at whatever 
  motor(2,PWM2);
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

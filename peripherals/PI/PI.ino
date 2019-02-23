#include "DualMC33926MotorShield.h"
#include <Encoder.h>
DualMC33926MotorShield md;

#define m1Dir 7
#define m1PWM 9
#define m2Dir 8
#define m2PWM 10
#define encA 2
#define encB 3

Encoder rotary(encA,encB);

float ki = 9.76;            //ki and kp just constants from our modeling
float kp = 0.5474;
float setVelocity = 4.0;    //looking for 4 radians per second (about what our motor rotated at when we were modeling our motor)
float scale = 21.65;        //scale = 255/(11.87 rad/s)    looked at our angular velocity at full speed to calculate the scale
float error = 0.0;          //start error and other variables at 0 for initial conditions, will get updated later
float integral = 0.0;
float I = 0.0;
float P = 0.0;
float PWM = 0.0;
float velocity = 0.0;
float deltaTime = 0.0;

unsigned long timerOld = 0;
unsigned long countOld = 0;
unsigned long timerNew = 0;
unsigned long countNew = 0;

int countsPerRad = 509;     //about equal to 3200 counts per revolution divided by 2pi rad/rev


void setup() {
  Serial.begin(9600);
  md.init();  
}

void loop() {
  timerNew = millis();        //timerNew in milliseconds, increments throughout running of code
  countNew = rotary.read();   //set countNew to however many counts the encoder has done
  deltaTime = (timerNew - timerOld)/1000.0;   //convert time in ms to seconds
  if (deltaTime > 0) {        //only let velocity calculated if deltaTime is not 0, don't want to divide by 0
     velocity = ((float)(countNew - countOld)/countsPerRad)/deltaTime;   //get velocity in radians per second
  }
  else {
     velocity = 0.0;          //just make sure velocity is 0 for initial conditions
  }

  Serial.print("       velocity = ");       //using this to debug
  Serial.print(velocity);                   //print velocity out to 4 decimal places

  countOld = countNew;        //set counter and timer old values to the new values for reference
  timerOld = timerNew;
  
  error = setVelocity - velocity;    //just formulas from the PID control article
  integral = integral + error*deltaTime;
  I = integral*ki;
  P = error*kp;
  PWM = (I + P)*scale;
  
  motor(2,PWM);           //run motor at whatever 
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

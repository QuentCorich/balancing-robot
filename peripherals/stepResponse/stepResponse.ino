//Alec Richey and Braeden Lieberman

#include "DualMC33926MotorShield.h"
#include <Encoder.h>
DualMC33926MotorShield md;

#define m1Dir 7
#define m1PWM 9
#define m2Dir 8
#define m2PWM 10
#define encA 2
#define encB 3

Encoder rotary(2,3);

void setup() {
  Serial.begin(9600);
  md.init();
}

void loop() {
  rotary.write(0);             //reset encoder counts
  unsigned long timerOld = 0;  //variables initialized to 0 for initial conditions
  unsigned long timerNew = 0;
  unsigned long countOld = 0;
  unsigned long countNew = 0;
  float deltaTime = 0.0;
  float velocity = 0.0;
  int countsPerRad = 509;      //about 509 counts per 1 radian of revolution
  
  while(countOld <= 6400){     //turn wheel about 2 times, then reset everything
    unsigned long timer = millis();   //create timer and reference for 1ms sampling time
    unsigned long timerRef = timer;
    
    timerNew = millis();            //set timerNew to whatever time the program is at
    countNew = rotary.read();       //set countNew to however many counts the encoder has seen
    deltaTime = (timerNew - timerOld)/1000.0;   //put deltaTime into seconds
    
    if(timer < 1000) {        //step response.  Keep motor off until 1 second has passed
      motor(2,0);
    }
     else {
      motor(2,150);
    }

    if (deltaTime > 0){    //don't let velocity be calculated with a deltaTime = 0, don't want to divide by 0
       velocity = ((float)(countNew - countOld)/countsPerRad)/deltaTime;
    }
    else {
       velocity = 0.0;
    }

    Serial.print("Velocity = ");  //used to see what velocity is
    Serial.print(velocity);
    Serial.println();

    countOld = countNew;    //set "old" variables to what new ones equaled
    timerOld = timerNew;
    delay(timerRef + 1 - timer);  //sample at about 1ms
    
  }
  
  motor(2,0);  //send off command to motor
  
  delay(2500);  //delay 2.5 seconds, really get motor to stop
  
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

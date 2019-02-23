#include "DualMC33926MotorShield.h"
DualMC33926MotorShield md;

#define m1Dir 7
#define m1PWM 9
#define m2Dir 8
#define m2PWM 10


void setup() {
  Serial.begin(9600);
  md.init();
}

void loop() {
  motor(1,150);
  delay(2000);
  motor(1,-150);
  delay(2000);

}

void motor (int m, int pwm) {  //input m = 1 for m1, pwm between 0 and 255
  if(m == 1){
    if(pwm > 0){  //if pwm > 0, turn wheel forward direction at some speed
      digitalWrite(m1Dir,HIGH);
      analogWrite(m1PWM,pwm);
    }
    else if(pwm < 0){  //if pwm < 0, turn wheel reverse direction at some speed
      digitalWrite(m1Dir,LOW);
      analogWrite(m1PWM,pwm);
    }
    else{
      analogWrite(m1PWM,0);
    }
  }//end motor m1
  if(m == 2){
    if(pwm > 0){  //if pwm > 0, turn wheel forward direction at some speed
      digitalWrite(m2Dir,HIGH);
      analogWrite(m2PWM,pwm);
    }
    else if(pwm < 0){  //if pwm < 0, turn wheel reverse direction at some speed
      digitalWrite(m2Dir,LOW);
      analogWrite(m2PWM,pwm);
    }
    else{
      analogWrite(m2PWM,0);
    }//end motor m2
  }
}


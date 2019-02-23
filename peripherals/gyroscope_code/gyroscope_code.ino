/*
 * Authors: Quentin Corich, Austin Hutchison
 * 
 * Angular velocity measured in rad/s
 * using SparkFun Gyro Breakout - LPY503AL 
 * 
 * Sources:
 * https://www.sparkfun.com/products/11341
 */

//define pins
#define x_axis A8
#define z_axis A9
#define HP 14 //High pass filter reset
#define PD 15 //Power down
#define ST 16 //Self Test
#define zeroValue 1230.0 //mV

//global variables
double x_input = 0;
double z_input = 0;
double x_offset = -.03;
double z_offset = -.03;

//radian = (pi/180) * degree

//sensitivity : 8.3 mV/(degree/s)

void setup() {

  Serial.begin(9600);
  gyroscopeInitilization();
  setOffset();
}

void loop() {
  
  getAngularVelocity();
  
  printValues();
  
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
  Serial.println(z_input, 6);
}


/****
 * Program Name: ToF_Code
 * Synopsis: This code is used to control the motors based on Time of Fligh data
 * There are 2 Tof sensors (left and right)
 * Created On: 2/5/22
 * Last Modified: 2/17/22
 ****/

#include <Wire.h>
#include <VL53L1X.h>

// define the PWM pins used for the left and right motors
const int leftMotor = 4; // the number of the left Vibration Module pin (PWM)
const int rightMotor = 5; // the number of the right Vibration Module pin (PWM)

// declare 2 tof sensors
VL53L1X leftTof;
VL53L1X rightTof;

// left and right Tof distances
int leftTofDist = 0;
int rightTofDist = 0;

void setup() {
  // initalize pins for the ToF sensors
  Serial.begin(115200);
  pinMode(47, OUTPUT);
  pinMode(53, OUTPUT);

  digitalWrite(47, LOW);
  digitalWrite(53, LOW);

  // initialize I2C communication
  delay(500);
  Wire.begin();

  pinMode(47, INPUT);
  delay(150);
  Serial.println("00");
  leftTof.init(true);

  Serial.println("01");
  delay(100);
  leftTof.setAddress(0x28);
  leftTof.setTimeout(100);
  Serial.println("02");

  pinMode(53,INPUT);
  delay(150);
  rightTof.init(true);
  Serial.println("03");
  delay(100);
  rightTof.setAddress(0x31);
  rightTof.setTimeout(100);
  Serial.println("04");
  
  leftTof.setDistanceMode(VL53L1X::Long);
  leftTof.setMeasurementTimingBudget(50000);
  leftTof.startContinuous(50); // continuous reading every 50 ms
  

  rightTof.setDistanceMode(VL53L1X::Long);
  rightTof.setMeasurementTimingBudget(50000);
  rightTof.startContinuous(50); // continuous reading every 50 ms

  
  delay(150);

  Serial.println("addresses set");
  Serial.println("I2C scanner. Scanning ...");

  byte count = 0;

  for (byte i = 1; i < 120; i++){
     Wire.beginTransmission(i);
     if (Wire.endTransmission() == 0){
     /* Only use following code for testing purposes */
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
    } 
  } 
  /*Testing purposes only */
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");

}

void loop() {
  // find distance read from the Tof sensors
  leftTofDist = readLeftTof(); 
  rightTofDist = readRightTof();
  Serial.print("Left ToF distance: ");
  Serial.println(leftTofDist);

  Serial.print("Right ToF distance: ");
  Serial.println(rightTofDist);

//  // generate output based on the distances read
//  if (leftTofDist > 0) {
//    leftTofMotorOutput(leftTofDist);
//  }
  
//  if (rightTofDist > 0) {
//    rightTofMotorOutput(rightTofDist);
//  }
//
}

// read data from left sensor
// returns: distance (in mm)
int readLeftTof(){
  int distance;
  distance = leftTof.read();
  if (distance < 609 || distance > 915) {
    distance = -1; // invalid range
  }
return distance; 
}

// read data from right sensor
// returns: distance (in mm)
int readRightTof(){
  int distance;
  distance = rightTof.read();
  if (distance < 609 || distance > 915) {
    distance = -1; // invalid range
  }
return distance; 
}

// in this function output from the left motor is produced based on the distance reading from the left time of flight sensor
// distance: the distance read by the sensor in mm
// linear equation that evaluates the strength of the motors PWM = -0.833*distance + 763 
void leftTofMotorOutput(int distance) {
  int PWM = -0.833 * distance + 763; 
  // int PWM = (-10 * distance - 760)/8.333; // inv of linear funct
  PWM = map(PWM, 609.6, 914.4, 1, 255); // map funct
  analogWrite(leftMotor, PWM);
// Serial.print("current motor output =");
// Serial.print(PWM);
// Serial.print(" "); 
}

// in this function output from the right motor is produced based on the distance reading from the right time of flight sensor
// distance: the distance read by the sensor in mm
// linear equation that evaluates the strength of the motors PWM = -0.833*distance + 763 
void rightTofMotorOutput(int distance) {
  int PWM = -0.833 * distance + 763; 
  // int PWM = (-10 * distance - 760)/8.333; // inv of linear funct
  PWM = map(PWM, 609.6, 914.4, 1, 255); // map funct
  analogWrite(rightMotor, PWM);
// Serial.print("current motor output =");
// Serial.print(PWM);
// Serial.print(" "); 
}

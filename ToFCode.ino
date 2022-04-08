/****
 * Program Name: ToF_Code
 * Synopsis: This code is used to control the motors based on Time of Flight data
 * There are 2 Tof sensors (left and right)
 * Created On: 2/5/22
 * Last Modified: 3/2/22
 ****/

#include <Wire.h>
#include <VL53L1X.h>

// define the PWM pins used for the left and right motors
//const int leftMotor = 4; // the number of the left Vibration Module pin (PWM)
//const int rightMotor = 5; // the number of the right Vibration Module pin (PWM)

// declare 2 tof sensors
VL53L1X leftTof;
//VL53L1X rightTof;

// left and right ToF reset pins
const int leftTofXshut = 2;
//const int rightTofXshut = 3;

// left and right Tof distances
int leftTofDist;
//int rightTofDist;

void setup() {
  while (!Serial) {} // loop until Serial initialized
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000); // 400 kHz I2C

  // Reset all ToF using XSHUT pins
  pinMode(leftTofXshut, OUTPUT);
  digitalWrite(leftTofXshut, LOW);
  
//  pinMode(rightTofXshut, OUTPUT);
//  digitalWrite(rightTofXshut, LOW);

  // Start up each sensor one at a time
  // left ToF
  pinMode(leftTofXshut, INPUT);
  delay(10);
  leftTof.setTimeout(500);
  if (!leftTof.init()) {
    Serial.println("Failed to detect and initialize left sensor");
  }
  leftTof.setAddress(0x2A);
  leftTof.startContinuous(50);
  
//  // right ToF
//  pinMode(rightTofXshut, INPUT);
//  delay(10);
//  rightTof.setTimeout(500);
//  if (!rightTof.init()) {
//    Serial.println("Failed to detect and initialize right sensor");
//  }
//  rightTof.setAddress(0x2B);
//  rightTof.startContinuous(50);
}

void loop() {
  // find distance read from the Tof sensors
  leftTofDist = leftTof.read();
  //rightTofDist = rightTof.read();

  Serial.print("left ToF distance: ");
  Serial.print(leftTofDist);
  Serial.println(" mm.");
//
//  Serial.print("right ToF distance: ");
//  Serial.print(rightTofDist);
//  Serial.println(" mm.");

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

//// read data from right sensor
//// returns: distance (in mm)
//int readRightTof(){
//  int distance;
//  distance = rightTof.read();
//  if (distance < 609 || distance > 915) {
//    distance = -1; // invalid range
//  }
//return distance; 
//}


//// in this function output from the right motor is produced based on the distance reading from the right time of flight sensor
//// distance: the distance read by the sensor in mm
//// linear equation that evaluates the strength of the motors PWM = -0.833*distance + 763 
//void rightTofMotorOutput(int distance) {
//  int PWM = -0.833 * distance + 763; 
//  // int PWM = (-10 * distance - 760)/8.333; // inv of linear funct
//  PWM = map(PWM, 609.6, 914.4, 1, 255); // map funct
//  analogWrite(rightMotor, PWM);
//// Serial.print("current motor output =");
//// Serial.print(PWM);
//// Serial.print(" "); 
//}

// in this function output from the left motor is produced based on the distance reading from the sensors
// distance: the distance read by the sensor in cm
// linear equation that evaluates the strength of the motors PWM = -7.9375*distance + 731.25 
//void leftMotorOutput(int distance) {
//  int PWM = 0;
//  if (distance < 59 || distance > 92){
//    analogWrite(leftMotor,PWM);
//  } else {
//    PWM = -7.9375*distance + 731.25; 
//    // int PWM = (-10 * distance - 760)/8.333; // inv of linear funct
//    PWM = map(PWM, 1, 247, 1, 255); // map funct
//    analogWrite(leftMotor, PWM);
//  }
//  Serial.print("Motor Strength: ");
//  Serial.println(PWM);
//}

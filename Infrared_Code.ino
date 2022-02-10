/****
 * Program Name: Infrared_Code
 * Synopsis: This code is used to control the motors based on Infrared data
 * There are a total of 4 IR sensors: an upper pair (left and right) and a 
 * lower pair (left and right). The upper pair is facing away from the walker.
 * Created On: 2/8/22
 * Last Modified: 2/10/22
 ****/
#include <SharpIR.h>
// define left and right sensors for the upper and lower pairs
#define irUpLeft A0
#define irLowLeft A1
#define irUpRight A2
#define irLowRight A3

#define model SharpIR::GP2Y0A02YK0F // for IR motor being used

SharpIR upperLeftIr(model, irUpLeft);
SharpIR lowerLeftIr(model, irLowLeft);
SharpIR upperRightIr(model, irUpRight);
SharpIR  lowerRightIr(model, irLowRight);


// ir distances (in cm)
int upperLeftIrDist;
int lowerLeftIrDist;
int upperRightIrDist;
int lowerRightIrDist;
int actualIrDistance; // distance value that determines motor output

// define the PWM pins used for the left and right motors
const int leftMotor = 4; // the number of the left Vibration Module pin (PWM)
const int rightMotor = 5; // the number of the right Vibration Module pin (PWM)


void setup() {
  Serial.begin(57600); // To view the data from the IR sensors

  // initialize motors
  pinMode(leftMotor, OUTPUT); // Set digital pin as input
  analogWrite(leftMotor, 0); // Start with module off

  pinMode(rightMotor, OUTPUT); // Set digital pin as input
  analogWrite(rightMotor, 0); // Start with module off
}

void loop() {
  upperLeftIrDist = upperLeftIr.getDistance();
  lowerLeftIrDist = lowerLeftIr.getDistance();
  
  upperRightIrDist = upperRightIr.getDistance();
  lowerLeftIrDist = lowerRightIr.getDistance();
  
  delay (500); // delay for half a millisecond 
  
  // process the data: determine what is the closest obstacle and send output
  processLeftIrData(lowerLeftIrDist, upperLeftIrDist);
  processRightIrData(lowerLeftIrDist, upperLeftIrDist);
}

// This function determines what the closest obstacle is to the walker based on the
// two left Ir sensors. If one sensor reads an obstacle notably closer than the other then 
// that data is used to create the left alert as it is more pressing. Actual distance is sent
// the the motor for output 
// lowerDistance: the lower IR sensor distance
// upperDistance: the upper IR sensor distance
void processLeftIrData(int lowerDistance, int upperDistance) {
  // check if either sensor has distance reading in alert range
  if((lowerDistance >= 24 && lowerDistance <= 36) || (upperDistance >= 20 && upperDistance <= 40)) {
    // if the difference is less than 5 inches, average the distances and send the motor output
    if (abs(lowerDistance - upperDistance) <= 5) {
      actualIrDistance = (lowerDistance + upperDistance) / 2; 
    } else {
      actualIrDistance = min(lowerDistance,upperDistance);
    }
    // send output to left motor
    leftIrMotorOutput(actualIrDistance);
  } 
}

// This function determines what the closest obstacle is to the walker based on the
// two right Ir sensors. If one sensor reads an obstacle notably closer than the other then 
// that data is used to create the left alert as it is more pressing. Actual distance is sent
// the the motor for output  
// lowerDistance: the lower IR sensor distance
// upperDistance: the upper IR sensor distance
// return: leftIrMotorPWM: the left motor PWM based on left IR data
void processRightIrData(int lowerDistance, int upperDistance) {
  
  // check if either sensor has distance reading in alert range
  if((lowerDistance >= 24 && lowerDistance <= 36) || (upperDistance >= 24 && upperDistance <= 36)) {
    // if the difference is less than 5 inches, average the distances and send the motor output
    if (abs(lowerDistance - upperDistance) <= 5) {
      actualIrDistance = (lowerDistance + upperDistance) / 2; 
    } else {
      actualIrDistance = min(lowerDistance,upperDistance);
    }
    // send output to left motor
    rightIrMotorOutput(actualIrDistance);
  } 
}

// This function generates the output of the left motor based on the averaged distance 
// read by the upper and lower right sensors
// distance: the avaeraged distance value 
// linear equation that evaluates the strength of the motors PWM = -21.167*distance + 763 (distance in inches)
void leftIrMotorOutput (int distance) {
  int PWM = -21.167 * distance + 763;
  PWM = map(PWM, 24, 36, 1, 255); // minimum output this function handle is 1 PWM
  analogWrite(leftMotor, PWM);
}

// This function generates the output of the right motor based on the averaged distance 
// read by the upper and lower right sensors
// distance: the avaeraged distance value 
// linear equation that evaluates the strength of the motors PWM = -21.167*distance + 763 (distance in inches)
void rightIrMotorOutput (int distance) {
  int PWM = -21.167 * distance + 763;
  PWM = map(PWM, 24, 36, 1, 255); // minimum output this function handle is 1 PWM
  analogWrite(rightMotor, PWM);
}

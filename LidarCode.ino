/****
 * Program Name: LidarCode
 * Synopsis: This code is used to control the motors based on Lidar data
 * There are 2 Lidar sensors: a left and right sensor
 * Created On: 2/6/22
 * Last Modified: 2/22/22
 ****/
#include <SoftwareSerial.h>
#include <TFMPlus.h> // TFMini Plus Library

// define the PWM pins used for the left and right motors
const int leftMotor = 4; // the number of the left Vibration Module pin (PWM)
const int rightMotor = 5; // the number of the right Vibration Module pin (PWM)

// declare Lidar sensors
TFMPlus leftLidar;
TFMPlus rightLidar;

// Serial ports for lidars
SoftwareSerial leftLidarSerial(50, 51); //TX RX
SoftwareSerial rightLidarSerial(40, 41); //TX RX


// lidar distance (in cm)
int16_t leftLidarDist = 0;
int16_t rightLidarDist = 0;

//lidar flux and temperature (not using these values but they need to be set up to use the getdata funct.)

int16_t leftLidarFlux = 0;
int16_t rightLidarFlux = 0;

int16_t leftLidarTemp = 0;
int16_t rightLidarTemp = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // terminal serial
  delay(20);

  leftLidar.begin(&Serial2); // initialize serial
}

void loop() {
  // get distance readings
 int distance = 0;
 leftLidar.getData(leftLidarDist, leftLidarFlux, leftLidarTemp);
 rightLidar.getData(rightLidarDist, rightLidarFlux, rightLidarTemp);

 // determine if distances are valid
 leftLidarDist = calcLidarDist(leftLidarDist);
 rightLidarDist = calcLidarDist(rightLidarDist);

 // motor output
 leftLidarMotorOutput(leftLidarDist);
 rightLidarMotorOutput(rightLidarDist);
}

// check if distance is within hazard range
// lidarDistance: distacne read from lidar
// returns: distance in cm or negative value if unneeded reading
int calcLidarDist(int16_t lidarDistance) {
  int distance = (int)lidarDistance;
  if (distance < 60 || distance > 92) {
     distance = -1; // indicator that outside of alert range
  }
  return distance;
}

// in this function output from the left motor is produced based on the distance reading from the leftLidar sensor
// distance: the distance read by the sensor in cm
// linear equation that evaluates the strength of the motors PWM = -7.9375*distance + 731.25 
void leftLidarMotorOutput(int16_t distance) {
  int PWM = -7.9375 * distance + 731.25;
  Serial.print("PWM from linear equation: "); 
  Serial.println(PWM);
  // int PWM = (-10 * distance - 760)/8.333; // inv of linear funct
  if (distance < 0) {
    analogWrite(leftMotor, 0);
      Serial.print("current left motor output = ");
      Serial.print(0);
      Serial.print(" ");
  } else {
    PWM = map(PWM, 1, 247, 1, 255); // map funct
    analogWrite(leftMotor, PWM);
    Serial.print("current left motor output = ");
    Serial.print(PWM);
    Serial.print(" ");
  }
}

// in this function output from the left motor is produced based on the distance reading from the right Lidar sensor
// distance: the distance read by the sensor in cm
// linear equation that evaluates the strength of the motors PWM = -7.9375*distance + 731.25 
void rightLidarMotorOutput(int distance) {
  int PWM = -7.9375 * distance + 731.25;
  Serial.print("PWM from linear equation: "); 
  Serial.println(PWM);
  // int PWM = (-10 * distance - 760)/8.333; // inv of linear funct
  if (distance < 0) {
    analogWrite(leftMotor, 0);
      Serial.print("current right motor output = ");
      Serial.print(0);
      Serial.print(" ");
  } else {
    PWM = map(PWM, 1, 247, 1, 255); // map funct
    analogWrite(rightMotor, PWM);
    Serial.print("current right motor output = ");
    Serial.print(PWM);
    Serial.print(" ");
  }
}

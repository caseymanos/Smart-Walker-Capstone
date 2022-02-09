/****
 * Program Name: Infrared_Code
 * Synopsis: This code is used to control the motors based on Lidar data
 * There are 2 Lidar sensors: a left and right sensor
 * Created On: 2/6/22
 * Last Modified: 2/8/22
 ****/
#include <SoftwareSerial.h>

// define the PWM pins used for the left and right motors
const int leftMotor = 4; // the number of the left Vibration Module pin (PWM)
const int rightMotor = 5; // the number of the right Vibration Module pin (PWM)

// define left and right Serial pins
SoftwareSerial leftLidarSerial(13,14); // RX, TX
SoftwareSerial rightLidarSerial(50,51); // RX, TX

int leftLidarDist; // distance reading of the left lidar
int rightLidarDist; // distance reading of the right lidar

int leftLidarStrength; // signal strength of left Lidar
int rightLidarStrength; // signal strength of right Lidar

int leftLidarCheck; // variable used to check viability of data on left Lidar
int rightLidarCheck; // variable used to check viability of data on right Lidar

int leftLidarData[9]; // save data measured by left Lidar
int rightLidarData[9]; // save data measured by right Lidar

const int HEADER = 0x59; // frame header of data package its the same for both Lidars

void setup() {
  leftLidarSerial.begin(115200);
  rightLidarSerial.begin(115200);
}

void loop() {
  // process left and right lidar data
  leftLidarDist = calcLeftLidarDist(leftLidarData, leftLidarCheck);
  rightLidarDist = calcRightLidarDist(rightLidarData, rightLidarCheck);

  // if the data produced is within alert range: generate an output
  if (leftLidarDist > 0) {
    leftLidarMotorOutput(leftLidarDist);
  }
  if (rightLidarDist > 0) {
    rightLidarMotorOutput(rightLidarDist);
  }

}

// calculate the distance read by the sensor
// leftLidarData: frame that holds lidar data
// leftLidarCheck: used to hold data of the left lidar
// returns: distance in cm
int calcLeftLidarDist(int leftLidarData[], int& leftLidarCheck) {
  int distance;
  if (leftLidarSerial.available()) {
    if (leftLidarSerial.read() == HEADER) { // assess data package frame header 0x59
      leftLidarData[0] = HEADER;
      if (leftLidarSerial.read() == HEADER) {
        leftLidarData[1] = HEADER;
        for (int i = 2; i < 9; i++) { //save data in array
          leftLidarData[i] = leftLidarSerial.read(); 
        }
        leftLidarCheck = leftLidarData[0]+ leftLidarData[1] + leftLidarData[2] + leftLidarData[3] 
                         + leftLidarData[4] + leftLidarData[5] + leftLidarData[6] + leftLidarData[7];
        if (leftLidarData[8] == (leftLidarCheck & 0xff)) { // verify data
          distance = leftLidarData[2] + leftLidarData[3] * 256;
        }
      }
    }
  }
  if (distance < 60 || distance > 92) {
     distance = -1; // indicator that outside of alert range
  }
  return distance;
}

// calculate the distance read by the sensor
// rightLidarData: frame that holds lidar data
// rightLidarCheck: used to hold data of the right lidar
// returns: distance in cm
int calcRightLidarDist(int rightLidarData[], int& rightLidarCheck) {
  int distance;
  if (rightLidarSerial.available()) {
    if (rightLidarSerial.read() == HEADER) { // assess data package frame header 0x59
      rightLidarData[0] = HEADER;
      if (rightLidarSerial.read() == HEADER) {
        rightLidarData[1] = HEADER;
        for (int i = 2; i < 9; i++) { //save data in array
          rightLidarData[i] = rightLidarSerial.read(); 
        }
        rightLidarCheck = rightLidarData[0]+ rightLidarData[1] + rightLidarData[2] + rightLidarData[3] 
                         + rightLidarData[4] + rightLidarData[5] + rightLidarData[6] + rightLidarData[7];
        if (leftLidarData[8] == (leftLidarCheck & 0xff)) { // verify data
          distance = rightLidarData[2] + rightLidarData[3] * 256;
        }
      }
    }
  }
   if (distance < 60 || distance > 92) {
     distance = -1; // indicator that outside of alert range
  }
  return distance;
}

// in this function output from the left motor is produced based on the distance reading from the leftLidar sensor
// distance: the distance read by the sensor in cm
// linear equation that evaluates the strength of the motors PWM = -8.333*distance + 763 
void leftLidarMotorOutput(int distance) {
  int PWM = -8.333 * distance + 763; 
  // int PWM = (-10 * distance - 760)/8.333; // inv of linear funct
  PWM = map(PWM, 609.6, 914.4, 1, 255); // map funct
  analogWrite(leftMotor, PWM);
// Serial.print("current motor output =");
// Serial.print(PWM);
// Serial.print(" "); 
}

// in this function output from the right motor is produced based on the distance reading from the Ultrasonic sensor
// distance: the distance read by the sensor in cm
// linear equation that evaluates the strength of the motors PWM = -8.333*distance + 763 
void rightLidarMotorOutput(int distance) {
  int PWM = -8.333 * distance + 763; 
  // int PWM = (-10 * distance - 760)/8.333; // inv of linear funct
  PWM = map(PWM, 609.6, 914.4, 1, 255); // map funct
  analogWrite(rightMotor, PWM);
// Serial.print("current motor output =");
// Serial.print(PWM);
// Serial.print(" "); 
}

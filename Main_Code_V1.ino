/*************************************************
 * Created On: 2/3/22
 * This is the main program which generates the output of both motors based on they distances read by all sensors
 * The data received by all sensors is compared and is averaged and the output is produced from this data
 * Last Modified: 2/8/22 
 **************************************************/
#include <Wire.h>
#include <VL53L1X.h>
#include <SoftwareSerial.h>
#include <SharpIR.h>

// define the PWM pins used for the left and right motors
const int leftMotor = 4; // the number of the left Vibration Module pin (PWM)
const int rightMotor = 5; // the number of the right Vibration Module pin (PWM)

// ULTRASONIC VALUES
SoftwareSerial UltraSerial(11,10); // RX, TX
unsigned char data[4]={};
float ultraDist; // distance in mm

// TIME OF FLIGHT VALUES
// declare 2 tof sensors
VL53L1X leftTof;
VL53L1X rightTof;

// left and right Tof distances (in mm)
uint16_t leftTofDist; 
uint16_t rightTofDist;

// INFRARED SENSOR VALUES
// define left and right sensors for the upper and lower pairs
#define irUpLeft A0
#define irLowLeft A1
#define irUpRight A2
#define irLowRight A3

#define model 20150 // for IR motor being used

SharpIR upperLeftIr(irUpLeft, model);
SharpIR lowerLeftIr (irLowLeft, model);
SharpIR upperRightIr(irUpRight, model);
SharpIR  lowerRightIr(irLowRight, model);

// ir distances (in cm)
int upperLeftIrDist;
int lowerLeftIrDist;
int upperRightIrDist;
int lowerRightIrDist;

// LIDAR SENSOR VALUES
// define left and right Serial pins of Lidar sensors
SoftwareSerial leftLidarSerial(13,14); // RX, TX
SoftwareSerial rightLidarSerial(50,51); // RX, TX

int leftLidarDist; // distance reading of the left lidar (in cm) 
int rightLidarDist; // distance reading of the right lidar (in cm)

int leftLidarStrength; // signal strength of left Lidar
int rightLidarStrength; // signal strength of right Lidar

int leftLidarCheck; // variable used to check viability of data on left Lidar
int rightLidarCheck; // variable used to check viability of data on right Lidar

int leftLidarData[9]; // save data measured by left Lidar
int rightLidarData[9]; // save data measured by right Lidar

const int HEADER = 0x59; // frame header of data package its the same for both Lidars

void setup() {
  // serial monitor initialization
  Serial.begin(115200);
  
  // motor setup
  pinMode(leftMotor, OUTPUT); // Set digital pin as input
  analogWrite(leftMotor, 0); // Start with module off

  pinMode(rightMotor, OUTPUT); // Set digital pin as input
  analogWrite(rightMotor, 0); // Start with module off

  // Ultrasonic setup
  UltraSerial.begin(9600); // Ultrasonic serial communication

  // initalize pins for the ToF sensors
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);

  // initialize I2C communication
  delay(500);
  Wire.begin();
  Wire.beginTransmission(0x29);

  digitalWrite(3, HIGH);
  delay(150);
  rightTof.init();
  rightTof.setAddress(0x33);

  leftTof.setDistanceMode(VL53L1X::Long);
  leftTof.setMeasurementTimingBudget(50000);
  leftTof.startContinuous(50); // continuous reading every 50 ms
  leftTof.setTimeout(100);

  rightTof.setDistanceMode(VL53L1X::Long);
  rightTof.setMeasurementTimingBudget(50000);
  rightTof.startContinuous(50); // continuous reading every 50 ms
  rightTof.setTimeout(100);
  
  delay(150);

  byte count = 0;

  for (byte i = 1; i < 120; i++){
     Wire.beginTransmission (i);
     if (Wire.endTransmission () == 0){
      count++;
      delay (1);  // maybe unneeded?
    } 
  } 
  // IR setup
  // initialize the sensors
//  upperLeftIr.begin(A0);
//  upperRightIr.begin(A1);
//  lowerLeftIr.begin(A2);
//  lowerRightIr.begin(A3);

  //Lidar Setup
  // UART Comm for lidars
  leftLidarSerial.begin(115200);
  rightLidarSerial.begin(115200); 
}

void loop() {
  /* array to hold left distances (all in cm)
   *  index 0 = Ultrasonic
  *  index 1 = Tof
   *  index 2 = Upper IR
   *  index 3 = Lower IR
   *  index 4 = Lidar
  */
  int leftDistances[5]; 

  // same order as previous array
  int rightDistances[5];

  // left and right distances found from most notable obstacle
  int actualLeftDist;
  int actualRightDist;

  
  // process all the sensor's data
  // process Ultrasonic data
  ultraDist = readUltraData();
  leftDistances[0] = static_cast<int>(ultraDist/10);
  rightDistances[0] = static_cast<int>(ultraDist/10);

  // process ToF data
  // find distance read from the Tof sensors
  leftTofDist = readLeftTof(); 
  leftDistances[1] = leftTofDist/10;
  
  rightTofDist = readRightTof();
  rightDistances[1] = rightTofDist/10;
  
  // process IR data
  upperLeftIrDist = upperLeftIr.distance();
  lowerLeftIrDist = lowerLeftIr.distance();

  leftDistances[2] = upperLeftIrDist;
  leftDistances[3] = lowerLeftIrDist;
  
  upperRightIrDist = upperRightIr.distance();
  lowerRightIrDist = lowerRightIr.distance();

  rightDistances[2] = upperRightIrDist;
  rightDistances[3] = lowerRightIrDist;
  
  // process Lidar data
  leftLidarDist = calcLeftLidarDist(leftLidarData, leftLidarCheck);
  rightDistances[4] = leftLidarDist;
  
  rightLidarDist = calcRightLidarDist(rightLidarData, rightLidarCheck);
  rightDistances[4] = rightLidarDist;

  // compare the values received by all sensors that affect the left motor
  actualLeftDist = findMin(leftDistances);
  actualRightDist = findMin(rightDistances);

  //generate PWM outputs
  leftMotorOutput(actualLeftDist);
  rightMotorOutput(actualRightDist);
  
  delay(100); // short delay before next reading
}

// this function reads the ultrasonic sensor data and determines if it should produce and output
// returns: distance, the sensor readings
int readUltraData() {
  int distance;
  do{
    for(int i=0;i<4;i++){
       data[i] = UltraSerial.read();
    }
 } while(UltraSerial.read() == 0xff);
 UltraSerial.flush();
 
 if(data[0]==0xff){
  int sum;
  sum = (data[0] + data[1] + data[2])&0x00FF;
  if(sum == data[3]) {
    distance = (data[1]<<8) + data[2];
    if(distance < 609.6 || distance > 914.4){   
      Serial.println("Outside the required sensing range");
      distance = -1; // negative values indicate errors and will not be turned into output
    }
  }else{
    Serial.println("ERROR");
    distance = -1; 
  }
}
return distance;
}

// read data from left sensor
// returns: distance (in mm)
uint16_t readLeftTof(){
  uint16_t distance;
  distance = leftTof.read();
  if (distance < 609 || distance > 915) {
    distance = 0; // invalid range
  }
return distance; 
}

// read data from right sensor
// returns: distance (in mm)
uint16_t readRightTof(){
  uint16_t distance;
  distance = rightTof.read();
  if (distance < 609 || distance > 915) {
    distance = 0; // invalid range
  }
return distance; 
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

// find the smallest value within range of alerts
// distArr[]: array of the distances
// returns: actualDist: value that will determine the motor strength
int findMin(int distArr[]) {
  int actualDist;
  int minDist = 92; // largest possible value that would emit a motor output
  for (int i = 0; i < 5; i++) {
    if (distArr[i] > 92 || distArr[i] < 59){
      distArr[i] = 600; // done so the invalid values are not considered
    }
    minDist = min(minDist,distArr[i]);
  }
  actualDist = minDist;
  return actualDist;
}

// in this function output from the left motor is produced based on the distance reading from the leftLidar sensor
// distance: the distance read by the sensor in cm
// linear equation that evaluates the strength of the motors PWM = -8.333*distance + 763 
void leftMotorOutput(int distance) {
  int PWM = -8.333 * distance + 763; 
  // int PWM = (-10 * distance - 760)/8.333; // inv of linear funct
  PWM = map(PWM, 60.96, 91.44, 1, 255); // map funct
  analogWrite(leftMotor, PWM);
}

// in this function output from the right motor is produced based on the distance reading from the Ultrasonic sensor
// distance: the distance read by the sensor in cm
// linear equation that evaluates the strength of the motors PWM = -8.333*distance + 763 
void rightMotorOutput(int distance) {
  int PWM = -8.333 * distance + 763; 
  // int PWM = (-10 * distance - 760)/8.333; // inv of linear funct
  PWM = map(PWM, 60.96, 91.44, 1, 255); // map funct
  analogWrite(rightMotor, PWM);
 
}

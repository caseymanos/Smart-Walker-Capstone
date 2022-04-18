/*************************************************
   Created On: 2/3/22
   This is the main program which generates the output of both motors based on they distances read by all sensors
   The data received by all sensors is compared and is averaged and the output is produced from this data
   Last Modified: 4/18/22
 **************************************************/
#include <Wire.h>
#include <VL53L1X.h>
#include <SoftwareSerial.h>
#include <SharpIR.h>
#include <TFMPlus.h>

#define WINDOW_SIZE 5  // for moving average filter

// OBJECT DETECTION VALUES
/* Change min and max distances to arrays corresponding with each dist array value
  ie minDist[2] = min dist for upper ir
  Array Order:
  Ultra
  ToF
  Upper IR
  Lower IR
  Lidar
*/
int minDist[5] = {56,73,54,54,90};
int maxDist[5] = {84,92,133,75,117};

const bool leftObjectDetected = false;
const bool rightObjectDetected = false;

// MOTOR VALUES
const int leftMotor = 4; // the number of the left Vibration Module pin (PWM)
const int rightMotor = 5; // the number of the right Vibration Module pin (PWM)
int leftPower;
int rightPower;
// variables to track hallway detection
bool lHall = false;
bool rHall = false;
bool lFlag = false;
bool rFlag = false;
int leftHallCheck = 0;
int rightHallCheck = 0;

// timing interval for object detection
unsigned long objectPrevTime = 0;
const int objectPd = 50;

// motor output interval
int motorPd = 30;
unsigned long motorPrevTime = 0;

// ULTRASONIC VALUES
unsigned int ultraDist; // distance of ultrasonic (mm)
byte hdr, data_h, data_l, chksum; // bytes for ultra data packages

// timing interval for ultrasonic
unsigned long ultraPrevTime = 0;
const int ultraPd = 100;

// moving average ultra values
int ultraIndex = 0;
int ultraAverage = 0;
int ultraSum = 0;
int ultraReadings[WINDOW_SIZE];

// TIME OF FLIGHT VALUES
// declare 2 tof sensors
VL53L1X leftTof;
VL53L1X rightTof;

// values for tof moving averages
int leftTofIndex = 0;
int leftTofAverage = 0;
int leftTofSum = 0;
int leftTofReadings[WINDOW_SIZE];

int rightTofIndex = 0;
int rightTofAverage = 0;
int rightTofSum = 0;
int rightTofReadings[WINDOW_SIZE];

// timing interval for ToF
int tofPd = 40;
unsigned long tofPrevTime = 0;

// INFRARED SENSOR VALUES
// define left and right sensors for the upper and lower pairs
#define irUpLeft A0
#define irLowLeft A1
#define irUpRight A2
#define irLowRight A3
#define model SharpIR::GP2Y0A02YK0F // for IR sensor being used

SharpIR upperLeftIr(model, irUpLeft);
SharpIR lowerLeftIr (model, irLowLeft);
SharpIR upperRightIr(model, irUpRight);
SharpIR  lowerRightIr(model, irLowRight);

// ir distances (in cm)
int upperLeftIrDist = 0;
int lowerLeftIrDist = 0;
int upperRightIrDist = 0;
int lowerRightIrDist = 0;

// ir period and time
unsigned long irPrevTime = 0;
const int irPd = 40;

// ir upper left and right arrays
int upLeftReadings[WINDOW_SIZE];
int upRightReadings[WINDOW_SIZE];
int irIndex = 0;

// LIDAR SENSOR VALUES
// declare Lidar sensors
TFMPlus leftLidar;
TFMPlus rightLidar;

// lidar distance (in cm)
int16_t leftLidarDist = 0;
int16_t rightLidarDist = 0;

// lidar moving average values
int leftLidarIndex = 0;
int leftLidarAverage = 0;
int leftLidarSum = 0;
int leftLidarReadings[WINDOW_SIZE];

int rightLidarIndex = 0;
int rightLidarAverage = 0;
int rightLidarSum = 0;
int rightLidarReadings[WINDOW_SIZE];

// timing interval for lidars
unsigned long lidarPrevTime = 0;
int lidarPd = 15;

/* arrays to hold left and right distances (all in cm)
    index 0 = Ultrasonic
    index 1 = Tof
    index 2 = Upper IR
    index 3 = Lower IR
    index 4 = Lidar
*/
int leftDistances[5];
int rightDistances[5];

void setup() {
  // motor setup
  pinMode(leftMotor, OUTPUT); // Set digital pin as input
  pinMode(rightMotor, OUTPUT); // Set digital pin as input

  analogWrite(leftMotor, 0); // Start with module off
  analogWrite(rightMotor, 0); // Start with module off

  // Ultrasonic setup
  Serial1.begin(9600); // Ultrasonic serial communication

  // initalize pins for the ToF sensors
  Wire.begin();
  Wire.setClock(400000); // 400 kHz I2C

  // Reset all ToF using XSHUT pins
  pinMode(47, OUTPUT);
  digitalWrite(47, LOW);

  pinMode(53, OUTPUT);
  digitalWrite(53, LOW);

  // Start up each sensor one at a time
  // left ToF
  pinMode(47, INPUT);
  delay(50);
  leftTof.setTimeout(500);
  while (!leftTof.init()) {
  }
  leftTof.setAddress(0x2A);
  leftTof.setDistanceMode(VL53L1X::Short);
  leftTof.startContinuous(50);

  // right ToF
  pinMode(53, INPUT);
  delay(10);
  rightTof.setTimeout(500);
  while (!rightTof.init()) {
  }
  rightTof.setAddress(0x2B);
  rightTof.setDistanceMode(VL53L1X::Short);
  rightTof.startContinuous(50);

  //Lidar Setup
  // UART Comm for lidars
  Serial2.begin(115200);
  delay(20);
  Serial3.begin(115200);
  leftLidar.begin(&Serial2);
  rightLidar.begin(&Serial3);
}

void loop() {
  // update each sensor/ motor once they have gotten a new data retreival period
  if ((unsigned long)(millis() - ultraPrevTime) >= ultraPd) {
     ultrasonicSensorCall();
  }
  if ((unsigned long)(millis() - irPrevTime) >= irPd) {
    infraredSensorCall();
  }

  if ((unsigned long)(millis() - tofPrevTime) >= tofPd) {
     ToFSensorCalls();
  }

  if ((unsigned long)(millis() - lidarPrevTime) >= lidarPd) {
    lidarSensorCall();
  }

  if ((unsigned long)(millis() - objectPrevTime) >= objectPd) {
      objectDetection(leftDistances, rightDistances); 
  }

  if ((unsigned long)(millis() - motorPrevTime) >= motorPd) {
       motorOutput(leftPower, lHall, leftMotor);
       motorOutput(rightPower, rHall, rightMotor);
  }
}

/* this function reads the data package transmitted by the ultrasonic sensor 
   the bytes are combined and converted from HEX mto DEC to become a distance
   reading.
   updates: leftDistances[], rightDistances[], ultraPrevTime
*/
void ultrasonicSensorCall() {
  if (Serial1.available()) {
    hdr = (byte)Serial1.read();
    if (hdr == 255) {
      data_h = (byte)Serial1.read();
      data_l = (byte)Serial1.read();
      chksum = (byte)Serial1.read();
      if (chksum == ((hdr + data_h + data_l) & 0x00FF)) {

        ultraDist = data_h * 256 + data_l;
        leftDistances[0] = movingAverage(ultraIndex, ultraAverage, ultraSum, ultraDist / 10, ultraReadings, WINDOW_SIZE);
        rightDistances[0] = movingAverage(ultraIndex, ultraAverage, ultraSum, ultraDist / 10, ultraReadings, WINDOW_SIZE);
      }
    }
  }
  else {
  }
  ultraPrevTime = millis();
}

/* this function gets the distance readings of all four infrared sensors
 * these distances is based on the average of 10 voltage readings in a 
 * moving average filter
 * updates: leftDistances[], rightDistances[], irPrevTime, irIndex, 
 * upLeftReadings, upRightReadings
*/
void infraredSensorCall() {
  upperLeftIrDist = upperLeftIr.getDistance();
  lowerLeftIrDist = lowerLeftIr.getDistance();
  upperRightIrDist = upperRightIr.getDistance();
  lowerRightIrDist = lowerRightIr.getDistance();

  leftDistances[2] = upperLeftIrDist;
  leftDistances[3] = lowerLeftIrDist;
  rightDistances[2] = upperRightIrDist;
  rightDistances[3] = lowerRightIrDist;

  upLeftReadings[irIndex] = upperLeftIrDist;
  upRightReadings[irIndex] = upperRightIrDist;
  irIndex = (irIndex + 1) % WINDOW_SIZE;

  irPrevTime = millis();
}


/* this function gets the distance readings of the time of flight sensors
 * updates: leftDistances[], rightDistances[], tofPrevTime 
*/
void ToFSensorCalls() {
  //Left sensor call
  leftTof.read();
  leftDistances[1] = movingAverage(leftTofIndex, leftTofAverage, leftTofSum, (leftTof.ranging_data.range_mm) / 10, leftTofReadings, WINDOW_SIZE);

  rightTof.read();
  rightDistances[1] = movingAverage(rightTofIndex, rightTofAverage, rightTofSum, (rightTof.ranging_data.range_mm) / 10, rightTofReadings, WINDOW_SIZE);
  tofPrevTime = millis();
}

/* calculates the average of an array of distance readings from each sensor
 *  index: index of newest value for distance readings array
 *  average: the average of the values in the readings array
 *  sum: sum of the values in the distance array
 *  readings: array of the sensor's distance readings
 *  wSize: size of the array, and the number of distance readings being averaged
 *  returns: average
 */
int movingAverage(int &index, int &average, int &sum, int value, int readings[], int wSize) {
  sum = sum - readings[index];
  readings[index] = value;
  sum = sum + value;
  index = (index + 1) % wSize;
  average = sum / wSize;
  return average;
}

/* this function gets the distance readings of the lidar sensors
 * updates: leftDistances[], rightDistances[], lidarPrevTime 
*/
void lidarSensorCall() {
  if ( leftLidar.getData(leftLidarDist)) // Get data from the device.
  {
    leftDistances[4] = movingAverage(leftLidarIndex, leftLidarAverage, leftLidarSum, leftLidarDist, leftLidarReadings, WINDOW_SIZE); // add distance to main array]
  }

  if ( rightLidar.getData(rightLidarDist)) // Get data from the device.
  {
    rightDistances[4] = movingAverage(rightLidarIndex, rightLidarAverage, rightLidarSum, rightLidarDist, rightLidarReadings, WINDOW_SIZE);
  }
  lidarPrevTime = millis();
}

/* takes the array of left and right distances and calculates the motor's output 
 *  based on the closest obstaclwe within the required range
 *  determines if the left or right upper IR sensors detect a hallway
 *  updates: lHall, lFlag, rHall, rFlag, leftPower, rightPower
 */
void objectDetection(int leftDistances[], int rightDistances[]) {
  /* arrays to hold left and right distances (all in cm)
      index 0 = Ultrasonic
      index 1 = Tof
      index 2 = Upper IR
      index 3 = Lower IR
      index 4 = Lidar
  */
  // Determine motor strength by shortest detected value
  int tempPower = 0;
  leftPower = 0;
  rightPower = 0;
  for (int j = 0; j < 5; j++) {
    // Test distances against preset arrays to
    if ((leftDistances[j] > minDist[j] && leftDistances[j] < maxDist[j] && j != 3) || (j==1 && (leftDistances[j] > minDist[j] && leftDistances[j] + 8 < maxDist[j]))) {
      tempPower = map(leftDistances[j], maxDist[j], minDist[j], 200, 255);
      if (tempPower > leftPower) {
        leftPower = tempPower;
      }
    }
    if ((rightDistances[j] > minDist[j] && rightDistances[j] < maxDist[j] && j != 3) || (j==1 && (rightDistances[j] > minDist[j] && rightDistances[j] < (maxDist[j] +9 )))) {
      tempPower = map(rightDistances[j], maxDist[j], minDist[j], 200, 255);
      if (tempPower > rightPower) {
        rightPower = tempPower;
      }
    }
  }
  if (leftDistances[2] > minDist[2] && leftDistances[2] < maxDist[2]) {
    lFlag = true;
  }
  if (rightDistances[2] > minDist[2] && rightDistances[2] < maxDist[2]) {
    rFlag = true;
  }

  /* Hallway detection: if object was detected from right or left and no longer was, set trigger to beep twice

  */
  if (leftDistances[2] > maxDist[2] && lFlag && hallwayDetection(upLeftReadings, leftDistances, lFlag)) {
    lHall = true;
    lFlag = false;
  }
  if (rightDistances[2] > maxDist[2] && rFlag && hallwayDetection(upRightReadings, rightDistances, rFlag)) {
    rHall = true;
    rFlag = false;
  }
  objectPrevTime = millis();
}

/* Determines if the sensor passed a hallway. Checks if there was a drop in readings 
 *  and the drop is sustained fror multiple data collections to determine if it is a hallway
 *  readings[]: the previous readings of the upper IR sensor
 *  distances: the most current readings of the sensor to date
 *  flag: bool to determine if the sensor is in range before a drop off of distances
 *  retunrs: widePath
 */
bool hallwayDetection(int readings[], int distances[], bool flag){
  bool widePath = false;
  if((distances[2] - readings[irIndex-3]) >= 45 && flag){
    widePath = true;
  }
  return widePath;
}
/* sends output to motors; either a solid output or a pattern if a hallway
 *  updates: hall, motorPrevTime
 */
void motorOutput(int &power, bool &hall, int motor) {
  hall = false;
  if (hall) {
    hall = false;
    for (int i = 0; i < 5; i++) {
      analogWrite(motor, 255);
      delay(150);
      analogWrite(motor, 0);
      delay(150);
      analogWrite(motor, 255);
      delay(150);
      analogWrite(motor, 0);
    }
  } else {
    analogWrite(motor, power);
  }
  motorPrevTime = millis();
}

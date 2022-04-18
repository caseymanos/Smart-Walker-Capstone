/*************************************************
   Created On: 2/3/22
   This is the main program which generates the output of both motors based on they distances read by all sensors
   The data received by all sensors is compared and is averaged and the output is produced from this data
   Last Modified: 4/13/22
   For use with BETTER SERIAL PLOTTER
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
*/
//int minDist[5] = {56,73,73,54,54,90};
//int maxDist[5] = {84,93,99,133,75, 117};

int minDist[5] = {56,73,54,54,90};
int maxDist[5] = {84,99,133,75, 117};

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

unsigned long objectPrevTime = 0;
const int objectPd = 50;

// motor hall interval
int motorPd = 30;
unsigned long motorPrevTime = 0;

// ULTRASONIC VALUES
unsigned int ultraDist; // distance of ultrasonic (mm)
byte hdr, data_h, data_l, chksum; // bytes for ultra data packages
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
#define model SharpIR::GP2Y0A02YK0F // for IR motor being used

SharpIR upperLeftIr(model, irUpLeft);
SharpIR lowerLeftIr (model, irLowLeft);
SharpIR upperRightIr(model, irUpRight);
SharpIR  lowerRightIr(model, irLowRight);

// ir distances (in cm)
int upperLeftIrDist;
int lowerLeftIrDist = 0;
int upperRightIrDist;
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

// lidar flux and temperature (not using these values but they need to be set up to use the getdata funct.)

int16_t leftLidarFlux = 0;
int16_t rightLidarFlux = 0;

int16_t leftLidarTemp = 0;
int16_t rightLidarTemp = 0;

/* arrays to hold left and right distances (all in cm)
    index 0 = Ultrasonic
    index 1 = Tof
    index 2 = Upper IR
    index 3 = Lower IR
    index 4 = Lidar
*/
int leftDistances[5];
int rightDistances[5];
// timing interval value
unsigned long lidarPrevTime = 0;
int lidarPd = 15;

unsigned long prevPrint = 0;

void setup() {
  // serial monitor initialization
  Serial.begin(115200);

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

  Serial.println("Starting in 5");
  delay(1000);
  Serial.println("4");
  delay(1000);
  Serial.println("3");
  delay(1000);
  Serial.println("2");
  delay(1000);
  Serial.println("1");
  delay(1000);
  
}

void loop() {

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
       leftMotorOutput(leftPower, lHall);
       rightMotorOutput(rightPower, rHall);
  }

      if ((unsigned long)(millis() - prevPrint) >= 100){
        for (int i = 0; i < 5; i++){
          Serial.print(leftDistances[i]);
          Serial.print("\t");
          Serial.print(rightDistances[i]);
          Serial.print("\t");
        }
        Serial.print(leftPower);
        Serial.print("\t");
        Serial.print(rightPower);
        Serial.print("\t");
        Serial.print(lHall);
        Serial.print("\t");
        Serial.println(rHall);
        prevPrint = millis();
      }
}


// Ultrasonic sensor function to run once
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

// Infrared Analog call
void infraredSensorCall() {
  upperLeftIrDist = upperLeftIr.getDistance();
  lowerLeftIrDist = lowerLeftIr.getDistance();
  upperRightIrDist = upperRightIr.getDistance();
  lowerRightIrDist = lowerRightIr.getDistance();

  leftDistances[2] = upperLeftIrDist;
  leftDistances[3] = lowerLeftIrDist;
  rightDistances[2] = upperRightIrDist;
  rightDistances[3] = lowerRightIrDist;

//  Serial.print(millis());
//  Serial.print("\t");
//  Serial.println(lowerLeftIrDist);

  upLeftReadings[irIndex] = upperLeftIrDist;
  upRightReadings[irIndex] = upperRightIrDist;
  irIndex = (irIndex + 1) % WINDOW_SIZE;

  irPrevTime = millis();
}


// ToF Sensor Call
//Places cm distances of L and R tof arrays in main dist array
void ToFSensorCalls() {
  //Left sensor call
  leftTof.read();
  leftDistances[1] = movingAverage(leftTofIndex, leftTofAverage, leftTofSum, (leftTof.ranging_data.range_mm) / 10, leftTofReadings, WINDOW_SIZE);

  rightTof.read();
  rightDistances[1] = movingAverage(rightTofIndex, rightTofAverage, rightTofSum, (rightTof.ranging_data.range_mm) / 10, rightTofReadings, WINDOW_SIZE);
  tofPrevTime = millis();
}

int movingAverage(int &index, int &average, int &sum, int value, int readings[], int wSize) {
  sum = sum - readings[index];
  readings[index] = value;
  sum = sum + value;
  index = (index + 1) % wSize;
  average = sum / wSize;
  return average;
}

void lidarSensorCall() {
  if ( leftLidar.getData(leftLidarDist)) // Get data from the device.
  {
    leftDistances[4] = movingAverage(leftLidarIndex, leftLidarAverage, leftLidarSum, leftLidarDist, leftLidarReadings, WINDOW_SIZE); // add distance to main array]
  }
  else                  // If the command fails...
  {
    leftLidar.printFrame();
  }

  if ( rightLidar.getData(rightLidarDist)) // Get data from the device.
  {
    rightDistances[4] = movingAverage(rightLidarIndex, rightLidarAverage, rightLidarSum, rightLidarDist, rightLidarReadings, WINDOW_SIZE);
  }
  else                  // If the command fails...
  {
    rightLidar.printFrame();
  }
  lidarPrevTime = millis();
}

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
    if ((leftDistances[j] > minDist[j] && leftDistances[j] < maxDist[j] && j != 3) || (j==1 && (leftDistances[j] > minDist[j] && leftDistances[j] < maxDist[j] + 6)) {
      tempPower = map(leftDistances[j], maxDist[j], minDist[j], 140, 255);
      if (tempPower > leftPower) {
        leftPower = tempPower;
                Serial.print("Left motor triggered by sensor: ");
                Serial.println(j);
      }
    }
    if (rightDistances[j] > minDist[j] && rightDistances[j] < maxDist[j] && j != 3) {
      tempPower = map(rightDistances[j], maxDist[j], minDist[j], 140, 255);
      if (tempPower > rightPower) {
        rightPower = tempPower;
                Serial.print("Right motor triggered by sensor: ");
                Serial.println(j);
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
    Serial.print("left Hallway bool triggered bc left hall sensor is: ");
    Serial.println(leftDistances[2]);
    lHall = true;
    lFlag = false;
  }
  if (rightDistances[2] > maxDist[2] && rFlag && hallwayDetection(upRightReadings, rightDistances, rFlag)) {
    Serial.print("right Hallway bool triggered bc right hall sensor is: ");
    Serial.println(rightDistances[2]);
    rHall = true;
    rFlag = false;
  }
  objectPrevTime = millis();
}

bool hallwayDetection(int readings[], int distances[], bool flag){
  bool widePath = false;
  if((distances[2] - readings[irIndex-3]) >= 45 && flag){
    widePath = true;
  }
  return widePath;
}

// in this function output from the left motor is produced based on objectDetection()
// distance: the distance read by the sensor in cm
// linear equation that evaluates the strength of the motors PWM = -7.9375*distance + 731.25 *old*
void leftMotorOutput(int &leftPower, bool &lHall) {
  lHall = false;
  if (lHall) {
    lHall = false;
    for (int i = 0; i < 5; i++) {
      analogWrite(leftMotor, 255);
      delay(150);
      analogWrite(leftMotor, 0);
      delay(150);
      analogWrite(leftMotor, 255);
      delay(150);
      analogWrite(leftMotor, 0);
    }
  } else {
    analogWrite(leftMotor, leftPower);
  }
}

// in this function output from the right motor is produced based on objectDetection()
// distance: the distance read by the sensor in cm
// linear equation that evaluates the strength of the motors PWM = -8.333*distance + 763 *old*
void rightMotorOutput(int &rightPower, bool &rHall) {
  rHall = false;
  if (rHall) {
    rHall = false;
    for (int i = 0; i < 5; i++) {
      analogWrite(rightMotor, 255);
      delay(150);
      analogWrite(rightMotor, 0);
      delay(150);
      analogWrite(rightMotor, 255);
      delay(150);
      analogWrite(rightMotor, 0);
    }
  } else {
    analogWrite(rightMotor, rightPower);
  }
  motorPrevTime = millis();
}

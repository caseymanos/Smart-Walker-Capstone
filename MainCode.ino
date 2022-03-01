/*************************************************
 * Created On: 2/3/22
 * This is the main program which generates the output of both motors based on they distances read by all sensors
 * The data received by all sensors is compared and is averaged and the output is produced from this data
 * Last Modified: 3/1/22 
 **************************************************/
#include <Wire.h>
#include <VL53L1X.h>
#include <SoftwareSerial.h>
#include <SharpIR.h>
#include <TFMPlus.h>

// MOTOR VALUES
  const int leftMotor = 4; // the number of the left Vibration Module pin (PWM)
  //const int rightMotor = 5; // the number of the right Vibration Module pin (PWM)

// ULTRASONIC VALUES
  unsigned int ultraDist; // distance of ultrasonic (mm)
  byte hdr, data_h, data_l, chksum; // bytes for ultra data packages
  

// TIME OF FLIGHT VALUES
  // declare 2 tof sensors
  VL53L1X leftTof;
 // VL53L1X rightTof;

  // left and right Tof distances (in mm)
  int leftTofDist; 
  //int rightTofDist;

// INFRARED SENSOR VALUES
  // define left and right sensors for the upper and lower pairs
  #define irUpLeft A0
//  #define irLowLeft A1
//  #define irUpRight A2
//  #define irLowRight A3
  #define model 20150 // for IR motor being used

  SharpIR upperLeftIr(irUpLeft, model);
//  SharpIR lowerLeftIr (irLowLeft, model);
//  SharpIR upperRightIr(irUpRight, model);
//  SharpIR  lowerRightIr(irLowRight, model);

  // ir distances (in cm)
  int upperLeftIrDist;
//  int lowerLeftIrDist;
//  int upperRightIrDist;
//  int lowerRightIrDist;

// LIDAR SENSOR VALUES
  // declare Lidar sensors
  TFMPlus leftLidar;
//  TFMPlus rightLidar;

  // lidar distance (in cm)
  int16_t leftLidarDist = 0;
//  int16_t rightLidarDist = 0;

  // lidar flux and temperature (not using these values but they need to be set up to use the getdata funct.)
  
  int16_t leftLidarFlux = 0;
//  int16_t rightLidarFlux = 0;
  
  int16_t leftLidarTemp = 0;
//  int16_t rightLidarTemp = 0;

void setup() {
  // serial monitor initialization
  Serial.begin(115200);
  Serial.println("Beginning setup");
  
  // motor setup
  pinMode(leftMotor, OUTPUT); // Set digital pin as input
  analogWrite(leftMotor, 0); // Start with module off

//  pinMode(rightMotor, OUTPUT); // Set digital pin as input
//  analogWrite(rightMotor, 0); // Start with module off

  // Ultrasonic setup
  Serial1.begin(9600); // Ultrasonic serial communication
  Serial.println("Ultrasonic set up.");

  // initalize pins for the ToF sensors
  pinMode(47, OUTPUT);
 // pinMode(53, OUTPUT);
  digitalWrite(47, LOW);
//  digitalWrite(53, LOW);

  // initialize I2C communication
  delay(500);
  Wire.begin();
  Wire.beginTransmission(0x29);

  digitalWrite(47, HIGH);
  delay(150);
//  rightTof.init();
//  rightTof.setAddress(0x33);

  leftTof.setDistanceMode(VL53L1X::Long);
  leftTof.setMeasurementTimingBudget(50000);
  leftTof.startContinuous(50); // continuous reading every 50 ms
  leftTof.setTimeout(100);

//  rightTof.setDistanceMode(VL53L1X::Long);
//  rightTof.setMeasurementTimingBudget(50000);
//  rightTof.startContinuous(50); // continuous reading every 50 ms
//  rightTof.setTimeout(100);
  
  delay(150);

  byte count = 0;

  for (byte i = 1; i < 120; i++){
     Wire.beginTransmission(i);
     if (Wire.endTransmission () == 0){
      count++;
      delay (1);  // maybe unneeded?
    } 
  }
  Serial.print("Set up ");
  Serial.print(count);
  Serial.println("sensors."); 

  //Lidar Setup
  // UART Comm for lidars
  Serial2.begin(115200);
  delay(20);
  //Serial3.begin(115200);
  leftLidar.begin(&Serial2);
 // rightLidar.begin(&Serial3);
}

void loop() {
  /* arrays to hold left and right distances (all in cm)
   *  index 0 = Ultrasonic
  *  index 1 = Tof
   *  index 2 = Upper IR
   *  index 3 = Lower IR
   *  index 4 = Lidar
  */
  int leftDistances[5]; 
 // int rightDistances[5];

  // left and right distances found from most notable obstacle
  int actualLeftDist;
 // int actualRightDist;
  
  // process all the sensor's data
  // process Ultrasonic data
  ultraDist = readUltraData();
  leftDistances[0] = static_cast<int>(ultraDist/10);
  Serial.print("Ultrasonic Distance: ");
  Serial.print(leftDistances[0]);
  Serial.println(" cm.");
//  rightDistances[0] = static_cast<int>(ultraDist/10);

  // process ToF data
  // find distance read from the Tof sensors
  leftTofDist = readLeftTof(); 
  leftDistances[1] = leftTofDist/10;
  Serial.print("Time of Flight Distance: ");
  Serial.print(leftDistances[1]);
  Serial.println(" cm.");
  
  //rightTofDist = readRightTof();
 // rightDistances[1] = rightTofDist/10;
  
  // process IR data
  upperLeftIrDist = upperLeftIr.getDistance();
//  lowerLeftIrDist = lowerLeftIr.getDistance();

  leftDistances[2] = upperLeftIrDist;
  Serial.print("Upperleft IR Distance: ");
  Serial.print(leftDistances[2]);
  Serial.println(" cm.");
  
//  leftDistances[3] = lowerLeftIrDist;
  leftDistances[3] = 0;
  
//  upperRightIrDist = upperRightIr.getDistance();
//  lowerRightIrDist = lowerRightIr.getDistance();

//  rightDistances[2] = upperRightIrDist;
//  rightDistances[3] = lowerRightIrDist;
  
  // process Lidar data
  leftDistances[4] = calcLidarDist(leftLidar,leftLidarDist, leftLidarFlux, leftLidarTemp);
  Serial.print("Lidar Distance: ");
  Serial.print(leftDistances[4]);
  Serial.println(" cm.");
  
//  rightDistances[4] = calcLidarDist(rightLidar,rightLidarDist, rightLidarFlux, rightLidarTemp);
  
  // compare the values received by all sensors that affect the left motor
  actualLeftDist = findMin(leftDistances);
//  actualRightDist = findMin(rightDistances);

  //generate PWM outputs
  leftMotorOutput(actualLeftDist);
//  rightMotorOutput(actualRightDist);
  
  delay(200); // short delay before next reading
}

// this function reads the ultrasonic sensor data and determines if it should produce and output
// returns: distance, the sensor reading from the ultrsonic in mm
unsigned int readUltraData() {
  unsigned int distance;
  if(Serial1.available()){
    hdr = (byte)Serial1.read();
    if (hdr == 255){
      data_h = (byte)Serial1.read();
      data_l = (byte)Serial1.read();
      chksum = (byte)Serial1.read();

      if (chksum == ((hdr + data_h + data_l)&0x00FF)){
        distance = data_h *256 + data_l;
      }
    }
  }

return distance;
}

// read data from left sensor
// returns: distance (in mm)
int readLeftTof(){
  int distance;
  distance = leftTof.read();
  if (distance < 609 || distance > 915) {
    distance = 0; // invalid range
  }
return distance; 
}

//// read data from right sensor
//// returns: distance (in mm)
//int readRightTof(){
//  int distance;
//  distance = rightTof.read();
//  if (distance < 609 || distance > 915) {
//    distance = 0; // invalid range
//  }
//return distance; 
//}

// calculate the distance read by the sensor
// lidar: TFMiniPlus sensor
// dist: distance reading (in cm)
// flux: strength of signal
// temp: temperature reading
// returns: distance reading
int16_t calcLidarDist(TFMPlus lidar, int16_t dist, int16_t flux, int16_t temp) {
  if( lidar.getData(dist, flux, temp)){ // Get data from the device. 
     // determine if distances are valid
    delay(20);
    if (dist < 60 || dist > 92) {
       dist = -1; // indicator that outside of alert range
    }
  }
  return dist;
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
// linear equation that evaluates the strength of the motors PWM = -7.9375*distance + 731.25 
void leftMotorOutput(int distance) {
  int PWM = 0;
  if (distance < 59 || distance > 92){
    analogWrite(leftMotor,PWM);
  } else {
    PWM = -7.9375*distance + 731.25; 
    // int PWM = (-10 * distance - 760)/8.333; // inv of linear funct
    PWM = map(PWM, 1, 247, 1, 255); // map funct
    analogWrite(leftMotor, PWM);
  }
  Serial.print("Motor Strength: ");
  Serial.println(PWM);
}

//// in this function output from the right motor is produced based on the distance reading from the Ultrasonic sensor
//// distance: the distance read by the sensor in cm
//// linear equation that evaluates the strength of the motors PWM = -8.333*distance + 763 
//void rightMotorOutput(int distance) {
//  int PWM = -8.333 * distance + 763; 
//  // int PWM = (-10 * distance - 760)/8.333; // inv of linear funct
//  PWM = map(PWM, 1, 247, 1, 255); // map funct
//  analogWrite(rightMotor, PWM);
// 
//}

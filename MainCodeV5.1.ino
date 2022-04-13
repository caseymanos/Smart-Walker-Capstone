/*************************************************
 * Created On: 2/3/22
 * This is the main program which generates the output of both motors based on they distances read by all sensors
 * The data received by all sensors is compared and is averaged and the output is produced from this data
 * Last Modified: 4/13/22 
 * Give longer delay to ultra to let it receive data faster
 **************************************************/
#include <Wire.h>
#include <VL53L1X.h>
#include <SoftwareSerial.h>
#include <SharpIR.h>
#include <TFMPlus.h>

// OBJECT DETECTION VALUES
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
  
// ULTRASONIC VALUES
  unsigned int ultraDist; // distance of ultrasonic (mm)
  byte hdr, data_h, data_l, chksum; // bytes for ultra data packages
  unsigned long previousTime = 0;
  const int ultraPd = 100;
  
// TIME OF FLIGHT VALUES
  // declare 2 tof sensors
  VL53L1X leftTof;
  VL53L1X rightTof;
  // left and right Tof distances (in mm)
  int ToFArray[2];

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
  int lowerLeftIrDist;
  int upperRightIrDist;
  int lowerRightIrDist;

// LIDAR SENSOR VALUES
  // declare Lidar sensors
  TFMPlus leftLidar;
  TFMPlus rightLidar;

  // lidar distance (in cm)
  int16_t leftLidarDist = 0;
  int16_t rightLidarDist = 0;

  // lidar flux and temperature (not using these values but they need to be set up to use the getdata funct.)
  
  int16_t leftLidarFlux = 0;
  int16_t rightLidarFlux = 0;
  
  int16_t leftLidarTemp = 0;
  int16_t rightLidarTemp = 0;
  
  /* arrays to hold left and right distances (all in cm)
   *  index 0 = Ultrasonic
  *  index 1 = Tof
   *  index 2 = Upper IR
   *  index 3 = Lower IR
   *  index 4 = Lidar
  */
  int leftDistances[5]; 
  int rightDistances[5];
  int minDist[5];
  int maxDist[5];
void setup() {
  // serial monitor initialization
  Serial.begin(115200);
  Serial.println("Beginning overall setup");
  
  // motor setup
  pinMode(leftMotor, OUTPUT); // Set digital pin as input
  pinMode(rightMotor, OUTPUT); // Set digital pin as input
  
  analogWrite(leftMotor, 0); // Start with module off
  analogWrite(rightMotor, 0); // Start with module off

  // Ultrasonic setup
  Serial1.begin(9600); // Ultrasonic serial communication
  Serial.println("Ultrasonic set up.");

  // initalize pins for the ToF sensors
  Wire.begin();
  Wire.setClock(400000); // 400 kHz I2C
  
  // Reset all ToF using XSHUT pins
  Serial.println("Beginning ToF setup");
  
  pinMode(47, OUTPUT);
  digitalWrite(47, LOW);
  
  pinMode(53, OUTPUT);
  digitalWrite(53, LOW);

  // Start up each sensor one at a time
  // left ToF
  pinMode(47, INPUT);
  delay(50);
  leftTof.setTimeout(500);
  while(!leftTof.init()) {
    Serial.println("Failed to detect and initialize left sensor");
  }
  Serial.println("Left sensor initialized.");
  leftTof.setAddress(0x2A);
  leftTof.startContinuous(50);
  
 // right ToF
  pinMode(53, INPUT);
  delay(10);
  rightTof.setTimeout(500);
  while(!rightTof.init()) {
    Serial.println("Failed to detect and initialize right sensor");
  }
  Serial.println("Right sensor initialized.");
  rightTof.setAddress(0x2B);
  rightTof.startContinuous(50); 

  //Lidar Setup
  // UART Comm for lidars
  Serial2.begin(115200);
  delay(20);
  Serial3.begin(115200);
  leftLidar.begin(&Serial2);
  rightLidar.begin(&Serial3);
  Serial.println("Lidar Finished Setup");
}

void loop() {

  if ((unsigned long)(millis() - previousTime) >= ultraPd){
    ultrasonicSensorCall(); 
  }
    infraredSensorCall();
    ToFSensorCalls();
    lidarSensorCall();
    objectDetection(leftDistances, rightDistances);
    leftMotorOutput(leftPower, lHall);
    rightMotorOutput(rightPower, rHall);
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

// Ultrasonic sensor function to run once
void ultrasonicSensorCall() {
 if (Serial1.available()) {
   previousTime = millis();
    hdr = (byte)Serial1.read();
    if (hdr == 255) {
      data_h = (byte)Serial1.read();
      data_l = (byte)Serial1.read();
      chksum = (byte)Serial1.read();
      if (chksum == ((hdr + data_h + data_l)&0x00FF)){
//        Serial.print(hdr);
//        Serial.print(",");
//        Serial.print(data_h);
//        Serial.print(",");
//        Serial.print(data_l);
//        Serial.print(",");
//        
//        Serial.print("=");
//      
//        Serial.print(hdr, HEX);
//        Serial.print(",");
//        Serial.print(data_h, HEX);
//        Serial.print(",");
//        Serial.print(data_l, HEX);
//        Serial.print(",");
//        Serial.print(chksum, HEX);
//        Serial.print(" => "); 
        ultraDist = data_h * 256 + data_l;
//        Serial.print(ultraDist, HEX);
//        Serial.print("=");
        Serial.print(ultraDist/10, DEC);
        Serial.print(" cm\t");
        Serial.println(previousTime);
        leftDistances[0] = ultraDist/10;
        rightDistances[0] = ultraDist/10;
      }
    }
  }
  else {
    Serial.println("Ultra not yet available.");
  }
}




// // process IR data
//  upperLeftIrDist = upperLeftIr.getDistance();
////  lowerLeftIrDist = lowerLeftIr.getDistance();
//
//  leftDistances[2] = upperLeftIrDist;
//  Serial.print("Upperleft IR Distance: ");
//  Serial.print(leftDistances[2]);
//  Serial.println(" cm.");
//  
////  leftDistances[3] = lowerLeftIrDist;
//  leftDistances[3] = 0;
//  
////  upperRightIrDist = upperRightIr.getDistance();
////  lowerRightIrDist = lowerRightIr.getDistance();
//
////  rightDistances[2] = upperRightIrDist;
////  rightDistances[3] = lowerRightIrDist;
//

// Infrared Analog call
 void infraredSensorCall() {
  //unsigned long delay1=millis();  // takes the time before the loop on the library begins

  upperLeftIrDist = upperLeftIr.getDistance();
  lowerLeftIrDist = lowerLeftIr.getDistance();
  upperRightIrDist = upperRightIr.getDistance();
  lowerRightIrDist = lowerRightIr.getDistance();

  leftDistances[2] = upperLeftIrDist;
  leftDistances[3] = lowerLeftIrDist;
  rightDistances[2] = upperRightIrDist;
  rightDistances[3] = lowerRightIrDist;
  
  Serial.print("upperLeftIR filtered distance: ");  // returns it to the serial monitor
  Serial.print(upperLeftIrDist);
  Serial.print("\t");

  Serial.print("lowerLeftIR filtered distance: ");  // returns it to the serial monitor
  Serial.print(lowerLeftIrDist);
  Serial.print("\t");
  
  Serial.print("upperRightIR filtered distance: ");  // returns it to the serial monitor
  Serial.print(upperRightIrDist);
  Serial.print("\t");
  
  Serial.print("lowerRightIR filtered distance: ");  // returns it to the serial monitor
  Serial.println(lowerRightIrDist);
 }


// ToF Sensor Call
//Places cm distances of L and R tof arrays in main dist array
void ToFSensorCalls(){
 
  //Left sensor call
  leftTof.read();
  Serial.print("Left ToF range: ");
  Serial.print(leftTof.ranging_data.range_mm);
  leftDistances[1] = (leftTof.ranging_data.range_mm)/10;
  Serial.print("\tstatus: ");
  Serial.print(VL53L1X::rangeStatusToString(leftTof.ranging_data.range_status));
  Serial.print("\tpeak signal: ");
  Serial.print(leftTof.ranging_data.peak_signal_count_rate_MCPS);
  Serial.print("\tambient: ");
  Serial.print(leftTof.ranging_data.ambient_count_rate_MCPS);
  Serial.println();
  
  //Right Tof sensor call
  rightTof.read();
  Serial.print("Right ToF range: ");
  Serial.print(rightTof.ranging_data.range_mm);
  rightDistances[1] = (rightTof.ranging_data.range_mm)/10;
  Serial.print("\tstatus: ");
  Serial.print(VL53L1X::rangeStatusToString(rightTof.ranging_data.range_status));
  Serial.print("\tpeak signal: ");
  Serial.print(rightTof.ranging_data.peak_signal_count_rate_MCPS);
  Serial.print("\tambient: ");
  Serial.print(rightTof.ranging_data.ambient_count_rate_MCPS);
  Serial.println();
}


 void lidarSensorCall() {
  
int16_t lidarLeftDist = 0;    // Distance to object in centimeters
int16_t lidarLeftFlux = 0;    // Strength or quality of return signal
int16_t lidarLeftTemp = 0;    // Internal temperature of Lidar sensor chip

int16_t lidarRightDist = 0;    
int16_t lidarRightFlux = 0;    
int16_t lidarRightTemp = 0;   

  if( leftLidar.getData( lidarLeftDist, lidarLeftFlux, lidarLeftTemp)) // Get data from the device.
      {
        Serial.print( "Dist: ");
        Serial.println(lidarLeftDist); // display distance,
        leftDistances[4] = lidarLeftDist; // add distance to main array
      }
      else                  // If the command fails...
      {
        leftLidar.printFrame();  // display the error and HEX dataa
      }

  if( rightLidar.getData( lidarRightDist, lidarRightFlux, lidarRightTemp)) // Get data from the device.
      {
       Serial.print( "Dist: ");
        Serial.println(lidarRightDist); // display distance,
        rightDistances[4] = lidarRightDist;
      }
      else                  // If the command fails...
      {
        rightLidar.printFrame();  // display the error and HEX dataa
      }
 }

void objectDetection(int leftDistances[], int rightDistances[]){
  /* arrays to hold left and right distances (all in cm)
   *  index 0 = Ultrasonic
  *  index 1 = Tof
   *  index 2 = Upper IR
   *  index 3 = Lower IR
   *  index 4 = Lidar
  */

  // print statements for debugging
  for (int i; i<5; i++){
    Serial.print("Index "+i);
    Serial.println(" left distance = "+ leftDistances[i]);
    Serial.println(" right distance = "+ rightDistances[i]);
  }
 
  /* Change min and max distances to arrays corresponding with each dist array value
  ie minDist[2] = min dist for upper ir
  */
  minDist[0]=10; // Ultrasonic
  minDist[1]=10; // ToF
  minDist[2]=10; // Upper Ir
  minDist[3]=10; // Lower IR
  minDist[4]=10; // Lidar 
  
  maxDist[0]=30; // Ultrasonic
  maxDist[1]=40; // ToF
  maxDist[2]=40; // Upper IR
  maxDist[3]=40; // Lower IR
  maxDist[4]=50; // Lidar

  // Determine motor strength by number of sensors detecting objects
  leftPower=0;
  rightPower=0;
  for (int j; j<5; j++){
    // Test distances against preset arrays to 
    if (leftDistances[j] > minDist[j] && leftDistances[j] < maxDist[j]) {
      leftPower += 1; }
    if (rightDistances[j] > minDist[j] && rightDistances[j] < maxDist[j]) {
      rightPower += 1; 
      }
    }
   // power strength varies from 0 to 255
   leftPower = leftPower*(255/5);
   rightPower = rightPower*(255/5);

  if (leftDistances[2] > minDist[2] && leftDistances[2] < maxDist[2]){
    lFlag = true;
  }
   if (rFlag = rightDistances[2] > minDist[2] && rightDistances[2] < maxDist[2]){
    rFlag = true;
   }
   
   /* Hallway detection: if object was detected from right or left and no longer was, set trigger to beep twice
    *  
    */
   if (leftDistances[2]> maxDist[2] && lFlag) {
     lHall = true;
     lFlag = false;
   }
   if (rightDistances[2] > maxDist[2]){
     rHall = true;
     rFlag = false;
   }
}

// in this function output from the left motor is produced based on objectDetection()
// distance: the distance read by the sensor in cm
// linear equation that evaluates the strength of the motors PWM = -7.9375*distance + 731.25 *old*
void leftMotorOutput(int leftPower, bool lHall) {
    if (lHall){
      lHall = false;
      analogWrite(leftMotor, HIGH);
      delay(25);
      analogWrite(leftMotor, LOW);
      delay(25);
      analogWrite(leftMotor, HIGH);
      delay(25);
      analogWrite(leftMotor, LOW);
    }
    
    analogWrite(leftMotor, leftPower);

}

// in this function output from the right motor is produced based on objectDetection()
// distance: the distance read by the sensor in cm
// linear equation that evaluates the strength of the motors PWM = -8.333*distance + 763 *old*
void rightMotorOutput(int rightPower,bool rHall) {
      if (rHall){
      rHall = false;
      analogWrite(rightMotor, HIGH);
      delay(25);
      analogWrite(rightMotor, LOW);
      delay(25);
      analogWrite(rightMotor, HIGH);
      delay(25);
      analogWrite(rightMotor, LOW);
    }
  analogWrite(rightMotor, rightPower);
}

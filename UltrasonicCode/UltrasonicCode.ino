/****
 * Program Name: Ultrasonic Code
 * Synopsis: This code is used to control the motors based on Ultrasnic data
 * Created On: 2/4/22
 * Last Modified: 2/8/22
 ****/

#include <SoftwareSerial.h>

 // Ultrasonic Values
 SoftwareSerial UltraSerial(11,10); // RX, TX
 unsigned char data[4]={};
 float ultraDist;

 // define the PWM pins used for the left and right motors
const int leftMotor = 4; // the number of the left Vibration Module pin (PWM)
const int rightMotor = 5; // the number of the right Vibration Module pin (PWM)

void setup() {
  Serial.begin(57600);  // serial monitor
  UltraSerial.begin(9600); // Ultrasonic serial communication
  
  pinMode(leftMotor, OUTPUT); // Set digital pin as input
  analogWrite(leftMotor, 0); // Start with module off

  pinMode(rightMotor, OUTPUT); // Set digital pin as input
  analogWrite(rightMotor, 0); // Start with module off
}

void loop() {
  // process Ultrasonic data and send PWM output
  int ultraDistance = readUltraData();
  
  // create output based on Ultrasonic data as long as distance is valid
  if (ultraDistance > 0) {
    leftUltraMotorOutput(ultraDistance);
    rightUltraMotorOutput(ultraDistance);
  }
  

}

// this function reads the ultrasonic sensor data and determines if it should produce and output
// returns: distance, the sensor readings
int readUltraData() {
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

// in this function output from the left motor is produced based on the distance reading from the Ultrasonic sensor
// distance: the distance read by the sensor in mm
// linear equation that evaluates the strength of the motors PWM = -0.833*distance + 763 
void leftUltraMotorOutput(int distance) {
  int PWM = -0.833 * distance + 763; 
  // int PWM = (-10 * distance - 760)/8.333; // inv of linear funct
  PWM = map(PWM, 609.6, 914.4, 1, 255); // map funct
  analogWrite(leftMotor, PWM);
// Serial.print("current motor output =");
// Serial.print(PWM);
// Serial.print(" "); 
}

// in this function output from the right motor is produced based on the distance reading from the Ultrasonic sensor
// distance: the distance read by the sensor in mm
// linear equation that evaluates the strength of the motors PWM = -0.833*distance + 763 
void rightUltraMotorOutput(int distance) {
  int PWM = -0.833 * distance + 763; 
  // int PWM = (-10 * distance - 760)/8.333; // inv of linear funct
  PWM = map(PWM, 609.6, 914.4, 1, 255); // map funct
  analogWrite(rightMotor, PWM);
// Serial.print("current motor output =");
// Serial.print(PWM);
// Serial.print(" "); 
}

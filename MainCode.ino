#include <Wire.h>
#include <VL53L1X.h>
#include <Average.h>
#include <SoftwareSerial.h>

// Variable and sensor declaration
VL53L1X leftToF;
VL53L1X rightToF;
Average<float> leftIRAve(10); // bucket for left IR sensor data
Average<float> rightIRAve(10); // bucket for right IR sensor data

void setup() {
  // put your setup code here, to run once:
  // Communcation Methods 
  // Infrared baud rate: 9600
  // Ultrasonic baud rate: 9600
  // Lidar baud rate: 115200 (but we should be able to alter it)
  // ToF Sensors buad rate:(115200)   

  // Turn on all sensors and Motors
  

  // (4) Infrared Sensors (GP2Y0A02YK0F)
  Serial.begin(9600); // main Serial port for Infrareds
       

  // (1) Ultrasonic Sensor (SEN0311)
  Serial1.begin(9600); // Serial1 for Ultrasonic

  // (2) Lidar Sensors (SJ-GU-TFmini-Plus-01)
  Serial2.begin(115200);

  // (2) ToF Sensors (VL53L1X)
  Serial3.begin(115200);
  Wire.begin();
  Wire.setClock(400000) // 400 kHz I2C
  sensor.setTimeout(500);
  if (!sensor.init()){
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(15000);
  sensor.startContinuous(15);
  Serial.println("new program");
  // (2) Motors (DFR0440)
  

}

void loop() {
  // put your main code here, to run repeatedly:

}

// ToF sensors device address is 0x52
// Will have to alter the device address of one of the sensors
// The max reading is 50,000 packets/sec
// each packet is 8 bits
byte i2c_readRegisterByte(uint8_t deviceAddress, uint_8 registerAddres) {
  byte registerData;
  Wire.beginTransmission(deviceAddress);  // set sensor target
  Wire.write(registerAddress);            // set memory pointer
  Wire.endTransmission();
}

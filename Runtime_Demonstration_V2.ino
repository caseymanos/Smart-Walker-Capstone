/*************************************************
 * 
 * Created On: 2/7/22
 * This is the program for the Runtime Demonstration Test
 * The code will run the sensors at 100% power the entire 12hr duration and vibrators at 100%, cycling on and off every five minutes for the 12hr duration.
 * 
 */

 /*
  * Need to program: 
  * 2 Vibration motors, pins 4, 13
  * 4 Infrared sensors Analog pins 
  * 2 ToF Distance sensors I2C pins ? ? SDA, SCL
  * 2 TF-Mini Plus sensors Serial2, Serial3
  * 1 Waterproof Ultrasonic sensor - Serial1
  */
 //Libraries
 #include <AltSoftSerial.h>
 #include <Wire.h>
 #include <VL53L1X.h>
 #include <SharpIR.h>
 #include <TFMPlus.h>  // Include TFMini Plus Library v1.5.0

 // Motor values
 const int motorPin1 = 4 ;  // the number of the first Vibration Module pin (PWM)
 const int motorPin2 = 13 ;  // the number of the second Vibration Module pin (PWM)
 const long motorInterval = 300000; // 5 minute interval
 unsigned long currentMillis = 0;
 unsigned long previousMotorIntervalMillis = 0; 
 byte powerMode;
 
  // Ultrasonic sensor 
unsigned char data[4]={};
float ultraSonicDistance;

// Infrared sensor 
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
int lowerRightIrDist;
int upperRightIrDist;
// ToF Distance sensor

VL53L1X leftTof;
VL53L1X rightTof;

// left and right Tof distances (in mm)
uint16_t leftTofDist; 
uint16_t rightTofDist;

// TF-Mini Plus values
TFMPlus lidarLeft;         // Create a TFMini Plus object
TFMPlus lidarRight;         // Create a TFMini Plus object

 void setup() {
  // Vibration Motor Setup
    powerMode = HIGH;
    pinMode(motorPin1, OUTPUT); // Set digital pin as output
    pinMode(motorPin2, OUTPUT); // Set digital pin as output
    analogWrite(motorPin1, LOW); // Start with module off
    analogWrite(motorPin2, LOW); // Start with module off
    Serial.begin(57600); // serial monitor
    Serial.println("Arduino Serial setup");
    Serial.println ("Arduino Motor setup");
    
  // Ultrasonic sensor setup
    Serial1.begin(57600); // Ultrasonic serial communication 
    Serial.println("Ultrasonic sensor setup");
  
  // TF-Mini setup

    Serial2.begin( 57600);  // Initialize TFMPLus device serial port.
    delay(20);               // Give port time to initalize
    lidarLeft.begin( &Serial2);   // Initialize device library object and...
                             // pass device serial port to the object.
   Serial3.begin( 57600);  // Initialize TFMPLus device serial port.
    delay(20);               // Give port time to initalize
    lidarRight.begin( &Serial3);   // Initialize device library object and...
                             // pass device serial port to the object.                          
    Serial.println("Lidar Setup");
  
  // Tof Distance setup
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
// what does this do
  for (byte i = 1; i < 120; i++){
     Wire.beginTransmission (i);
     if (Wire.endTransmission () == 0){
      count++;
      delay (1);  // maybe unneeded?
    } 
  } 
  }
 

 void loop() {
    // Turn on all sensors and motors for five minutes
    currentMillis = millis();
    motorsOutput();
    ultrasonicSensorCall();
    infraredSensorCall();
    ToFSensorCall();
    lidarSensorCall();    
  }

 // output from the motor is set to 100% or 0% based on 5 minute interval
 void motorsOutput() {
  if (currentMillis - previousMotorIntervalMillis >= motorInterval){
    powerMode = ! powerMode;
    analogWrite(motorPin1, powerMode); 
    analogWrite(motorPin2, powerMode);
    previousMotorIntervalMillis += motorInterval;
  }
 }

// Ultrasonic sensor function to run once
 void ultrasonicSensorCall() {
  
       do{
       for(int i=0;i<4;i++)
         {
           data[i]=Serial1.read();
         }
       }
    while(Serial1.read()==0xff);
    Serial1.flush();
    if(data[0]==0xff){
        int sum;
        sum=(data[0]+data[1]+data[2])&0x00FF;
          if(sum==data[3]) {
            ultraSonicDistance=(data[1]<<8)+data[2];
             Serial.print("Ultrasonic distance=");
             Serial.println(ultraSonicDistance/10);
             
             Serial.println("cm");
            }else{     
             Serial.println("Ultrasonic distance reading outside the preset limit");
              }
        }else {
          Serial.println("Ultrasonic Error");
        }
    }
 
 // ToFSensors
 void ToFSensorCall() {
  //Left sensor call
  leftTof.read();
  Serial.print("Left ToF range: ");
  Serial.print(leftTof.ranging_data.range_mm);
  Serial.print("\tstatus: ");
  Serial.print(VL53L1X::rangeStatusToString(leftTof.ranging_data.range_status));
  Serial.print("\tpeak signal: ");
  Serial.print(leftTof.ranging_data.peak_signal_count_rate_MCPS);
  Serial.print("\tambient: ");
  Serial.print(leftTof.ranging_data.ambient_count_rate_MCPS);

  //Right Tof sensor call
  rightTof.read();
  Serial.print("Right ToF range: ");
  Serial.print(rightTof.ranging_data.range_mm);
  Serial.print("\tstatus: ");
  Serial.print(VL53L1X::rangeStatusToString(rightTof.ranging_data.range_status));
  Serial.print("\tpeak signal: ");
  Serial.print(rightTof.ranging_data.peak_signal_count_rate_MCPS);
  Serial.print("\tambient: ");
  Serial.print(rightTof.ranging_data.ambient_count_rate_MCPS);
  Serial.println();
 }
 // Infrared Analog call
 void infraredSensorCall() {
  unsigned long delay1=millis();  // takes the time before the loop on the library begins

  upperLeftIrDist = upperLeftIr.distance();
  lowerLeftIrDist = lowerLeftIr.distance();
  upperRightIrDist = upperRightIr.distance();
  lowerRightIrDist = lowerRightIr.distance();

  Serial.print("upperLeftIR Mean distance: ");  // returns it to the serial monitor
  Serial.println(upperLeftIrDist);

  Serial.print("lowerLeftIR Mean distance: ");  // returns it to the serial monitor
  Serial.println(lowerLeftIrDist);
  
  Serial.print("upperRightIR Mean distance: ");  // returns it to the serial monitor
  Serial.println(upperRightIrDist);
  
  Serial.print("upperLeftIR Mean distance: ");  // returns it to the serial monitor
  Serial.print(upperLeftIrDist);
  Serial.print(" ; ");  // returns it to the serial monitor
  
  
  unsigned long delay2=millis()-delay1;  // the following gives you the time taken to get the measurement
  Serial.print("Time taken (ms): ");
  Serial.println(delay2);  
 }
 void lidarSensorCall() {
  
int16_t lidarLeftDist = 0;    // Distance to object in centimeters
int16_t lidarLeftFlux = 0;    // Strength or quality of return signal
int16_t lidarLeftTemp = 0;    // Internal temperature of Lidar sensor chip

int16_t lidarRightDist = 0;    
int16_t lidarRightFlux = 0;    
int16_t lidarRightTemp = 0;   

  if( lidarLeft.getData( lidarLeftDist, lidarLeftFlux, lidarLeftTemp)) // Get data from the device.
      {
        Serial.println( "Dist:%04icm ", lidarLeftDist);   // display distance,
        Serial.println( "Flux:%05i ",   lidarLeftFlux);   // display signal strength/quality,
        Serial.println( "Temp:%2i%s",  lidarLeftTemp, "C");   // display temperature,
        Serial.println( "\r\n");                   // end-of-line.
      }
      else                  // If the command fails...
      {
        lidarLeft.printFrame();  // display the error and HEX dataa
      }

  if( lidarRight.getData( lidarRightDist, lidarRightFlux, lidarRightTemp)) // Get data from the device.
      {
        Serial.println( "Dist:%04icm ",  lidarRightDist);   // display distance,
        Serial.println( "Flux:%05i ",   lidarRightFlux);   // display signal strength/quality,
        Serial.println( "Temp:%2i%s",  lidarRightTemp, "C");   // display temperature,
        Serial.println( "\r\n");                   // end-of-line.
      }
      else                  // If the command fails...
      {
        lidarRight.printFrame();  // display the error and HEX dataa
      }
 }
 

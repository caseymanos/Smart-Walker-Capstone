  /****
 * Program Name: Infrared_Code
 * Synopsis: This code is to try and get noise removed from the IR sensors using 
 * a moving average filter on the voltage 
 * Created On: 3/24/22
 * Last Modified: 4/7/22
 ****/
 
// values for voltage noise removal 
#define WINDOW_SIZE 10

int upLeftIrIndex = 0;
int lowLeftIrIndex = 0;
int upRightIrIndex = 0;
int lowRightIrIndex = 0;

int upLeftIrSum = 0;
int lowLeftIrSum = 0;
int upRightIrSum = 0;
int lowRightIrSum = 0;

int upLeftIr_Readings[WINDOW_SIZE];
int lowLeftIr_Readings[WINDOW_SIZE];
int upRightIr_Readings[WINDOW_SIZE];
int lowRightIr_Readings[WINDOW_SIZE];

int upLeftIrAveraged = 0;
int lowLeftIrAveraged = 0;
int upRightIrAveraged = 0;
int lowRightIrAveraged = 0;

// values for IR sensors
#include <SharpIR.h>
// define left and right sensors for the upper and lower pairs
// Analog pins
#define irUpLeft A0
#define irLowLeft A1
#define irUpRight A2
#define irLowRight A3

#define model SharpIR::GP2Y0A02YK0F // for IR motor being used

SharpIR upperLeftIr(model, irUpLeft);
SharpIR lowerLeftIr(model, irLowLeft);
SharpIR upperRightIr(model, irUpRight);
SharpIR lowerRightIr(model, irLowRight);

// ir distances (in cm)
float upperLeftIrDist;
float lowerLeftIrDist;
float upperRightIrDist;
float lowerRightIrDist;

void setup() {
  Serial.begin(9600); // To view the data from the IR sensors
  Serial.println("5 seconds");
  delay(1000);
  Serial.println("4 seconds");
  delay(1000);
  Serial.println("3 seconds");
  delay(1000);
  Serial.println("2 seconds");
  delay(1000);
  Serial.println("1 second");
  delay(1000);
}

void loop() {
  // Get voltages with moving average and then get distance
  upperLeftIrDist = IrMovingAverage(upLeftIrSum, upLeftIr_Readings, analogRead(irUpLeft), upLeftIrIndex, upLeftIrAveraged, upperLeftIrDist, upperLeftIr, irUpLeft);
  lowerLeftIrDist = IrMovingAverage(lowLeftIrSum, lowLeftIr_Readings, analogRead(irLowLeft), lowLeftIrIndex, lowLeftIrAveraged, lowerLeftIrDist, lowerLeftIr, irLowLeft);  
  upperRightIrDist = IrMovingAverage(upRightIrSum, upRightIr_Readings, analogRead(irUpRight), upRightIrIndex, upRightIrAveraged, upperRightIrDist, upperRightIr, irUpRight);
  lowerRightIrDist = IrMovingAverage(lowRightIrSum, lowRightIr_Readings, analogRead(irLowRight), lowRightIrIndex, lowRightIrAveraged, lowerRightIrDist, lowerRightIr, irLowRight);

  // print out distances  
  Serial.print(upperLeftIrDist);
  Serial.print("\t");
  Serial.print(upLeftIrAveraged);
  Serial.print("\t");
  Serial.print("\t");
  
  Serial.print(lowerLeftIrDist);
  Serial.print("\t");
  Serial.print(lowLeftIrAveraged);
  Serial.print("\t");
  Serial.print("\t");
  
  Serial.print(upperRightIrDist);
  Serial.print("\t");
  Serial.print(upRightIrAveraged);
  Serial.print("\t");
  Serial.print("\t");

  Serial.print(lowerRightIrDist);
  Serial.print("\t");
  Serial.print(lowRightIrAveraged);
  Serial.print("\t");
  Serial.println("\t");
  delay(100);
}

// function to apply moving average to distance readings
int IrMovingAverage(int &SUM, int IR_READINGS[WINDOW_SIZE], int VOLTAGE, int &INDEX, int &AVERAGED, float &DISTANCE, SharpIR SENSOR, int PIN){
  SUM = SUM - IR_READINGS[INDEX];                     // Remove the oldest entry from the sum
  VOLTAGE = analogRead(PIN);                          // Read the next sensor value
  IR_READINGS[INDEX] = VOLTAGE;                       // Add the newest reading to the window
  SUM = SUM + VOLTAGE;                                // Add the newest reading to the sum
  INDEX = (INDEX+1) % WINDOW_SIZE;                    // Increment the index, and wrap to 0 if it exceeds the window size
  AVERAGED = SUM / WINDOW_SIZE;                       // Divide the sum of the window by the window size for the result
  DISTANCE = SENSOR.getDistance(AVERAGED);
  return DISTANCE;
}

// used to correct the inaccuracies of the IR sensor readings
float IrCorrection(int pinNum, float distance, int voltage){
  float actual = distance;
  switch (pinNum) {
  case A0:
    distance = 8 * pow(10,-7) * pow(voltage,4) 
                        - 0.0005 * pow(voltage,3)
                        + 0.1238 * pow(voltage, 2)
                        - 12.88 * voltage
                        + 490.78 + actual;
  return distance;
  break;
  case A2:
    distance = 5 * pow(10,-7) * pow(voltage,4) 
                        - 0.0003 * pow(voltage,3)
                        + 0.0749 * pow(voltage, 2)
                        - 7.2863 * voltage
                        + 252.31 + actual;
  return distance;
  break;
  case A1:
    distance = 7 * pow(10,-7) * pow(voltage,4) 
                        - 0.0005 * pow(voltage,3)
                        + 0.1099 * pow(voltage, 2)
                        - 10.862 * voltage
                        + 378.03 + actual;
  return distance;
  break;

  case A3:
    distance = 9 * pow(10,-7) * pow(voltage,4) 
                        - 0.0005 * pow(voltage,3)
                        + 0.1256 * pow(voltage, 2)
                        - 12.062 * voltage
                        + 409.56 + actual;
  return distance;
  break;
    }
}

/****
 * Program Name: Infrared_Code
 * Synopsis: This code is to try and get noise removed from the IR sensors
 * Created On: 3/24/22
 * Last Modified: 3/24/22
 ****/
 
// values for noise removal 
#define WINDOW_SIZE 5

int INDEX = 0;
int SUM = 0;
int IR_READINGS[WINDOW_SIZE];
int AVERAGED = 0;

// values for IR sensor
#include <SharpIR.h>
// define left and right sensors for the upper and lower pairs
// Analog pins
#define irUpLeft A0

#define model SharpIR::GP2Y0A02YK0F // for IR motor being used

SharpIR upperLeftIr(model, irUpLeft);

// ir distances (in cm)
int upperLeftIrDist;

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
  SUM = SUM - IR_READINGS[INDEX];                        // Remove the oldest entry from the sum
  upperLeftIrDist = upperLeftIr.getDistance();        // Read the next sensor value
  IR_READINGS[INDEX] = upperLeftIrDist;                  // Add the newest reading to the window
  SUM = SUM + upperLeftIrDist;                        // Add the newest reading to the sum
  INDEX = (INDEX+1) % WINDOW_SIZE;                    // Increment the index, and wrap to 0 if it exceeds the window size

  AVERAGED = SUM / WINDOW_SIZE;             // Divide the sum of the window by the window size for the result

  Serial.print("Sensor 0 distance: ");
  Serial.println(AVERAGED);
  Serial.print("\t");
  Serial.print(" Analog pin voltage: ");
  Serial.print(analogRead(irUpLeft));
  Serial.println(" V");
  delay(25);
}

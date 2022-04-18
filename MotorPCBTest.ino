/* Testing motors on PCB
 * 
 */
const int leftMotor = 17; // the number of the left Vibration Module pin (PWM)
const int rightMotor = 15; // the number of the right Vibration Module pin (PWM)
void setup() {
  pinMode(leftMotor, OUTPUT); // Set digital pin as input
  pinMode(rightMotor, OUTPUT); // Set digital pin as input
  analogWrite(leftMotor, 0); // Start with module off
  analogWrite(rightMotor, 0); // Start with module off

}

void loop() {
  analogWrite(leftMotor, 255);
  delay(1000);
  analogWrite(leftMotor, 0);
  delay(1000);
  analogWrite(rightMotor, 255);
  delay(1000);
  analogWrite(rightMotor, 0);
  delay(1000);
}

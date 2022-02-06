/*************************************************
 * 
 * Created On: 2/3/22
 * This is the program for the Response Time Test
 * The code will receive data for one second, this data will create an output in the motors 
 * 
 */
 #include <SoftwareSerial.h>

 // Ultrasonic Values
 SoftwareSerial mySerial(11,10); // RX, TX
 unsigned char data[4]={};
 float distance;

 // Motor values
 const int VibPin = 4 ;  // the number of the Vibration Module pin (PWM)
 
 void setup() {
  pinMode(VibPin, OUTPUT); // Set digital pin as input
  analogWrite(VibPin, 0); // Start with module off
  
  Serial.begin(57600); // serial monitor
  mySerial.begin(9600); // Ultrasonic serial communication
 }

 void loop() {
     do{
     for(int i=0;i<4;i++)
     {
       data[i]=mySerial.read();
     }
  }while(mySerial.read()==0xff);

  mySerial.flush();

  if(data[0]==0xff){
      int sum;
      sum=(data[0]+data[1]+data[2])&0x00FF;
        if(sum==data[3]) {
          distance=(data[1]<<8)+data[2];
          if(distance>30 && distance < 915){
            motorOutput(distance);
            //Serial.print("distance=");
           Serial.println(distance/10);
           
           // Serial.println("cm");
          }else{     
           // Serial.println("Below outside the limit");
            }
      }else {
        Serial.println("ERROR");
      }
  }
     delay(100);
  }

 // in this function output from the motor is produced based on the distance reading
 // distance: the distance read by the sensor in mm
 // linear equation that evaluates the strength of the motors PWM = -8.333*distance + 763
 void motorOutput(int distance) {
  //int PWM = -8.333 * (distance / 10) + 763; 
  int PWM = (-10 * distance - 760)/8.333; // inv of linear funct
  PWM = map(PWM, 30, 915, 0, 255); // map funct
  analogWrite(VibPin, PWM);
// Serial.print("current motor output =");
  Serial.print(PWM);
  Serial.print(" "); 
 }

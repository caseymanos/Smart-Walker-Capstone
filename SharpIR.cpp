#include "SharpIR.h"
// EDIT (4/3): changed uint8_t to float for more precision in values 
// EDIT (4/3): included functions to create more accurate readings
// EDIT (4/5): moved correction formulas to Arduino code
// EDIT (4/9): i made getDist() contin ue to return as type in
//			   but now it returns mm instead of cm distance
int SharpIR::getDistance(bool avoidBurstRead) {
		int distance;
		float averaged;
		int voltage;

		if( !avoidBurstRead ) while( millis() <= lastTime + 20 ) {} //wait for sensor's sampling time

		lastTime = millis();

		switch( sensorType ) {
			case GP2Y0A41SK0F :

				distance = 2076/(analogRead(pin)-11);

				if(distance > 30) return 31;
				else if(distance < 4) return 3;
				else return distance;

				break;

			case GP2Y0A21YK0F :

				distance = 4800/(analogRead(pin)-20);

				if(distance > 80) return 81;
				else if(distance < 10) return 9;
				else return distance;

				break;

			case GP2Y0A02YK0F :
				sum = sum - readings[index];
				voltage = analogRead(pin);
				readings[index] = voltage;
				sum = sum + voltage;
				index = (index + 1) % WINDOW_SIZE;
				averaged = sum / WINDOW_SIZE;

				distance = 9462/(averaged - 16.92) * 100;
				if(distance > 15000) return 15100;
				else if(distance < 20) return 19;
				else return (int)distance;

				break;
		}
	}






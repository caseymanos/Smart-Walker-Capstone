/*
 * SharpIR
 * Library for the acquisition of distance data from Sharp IR sensors
 */

#pragma once

#include <Arduino.h>
#include <math.h>
#define WINDOW_SIZE 10

class SharpIR
	{
		public:

			using sensorCode = const uint8_t ;
			int index;
			int sum;
			int readings[WINDOW_SIZE];
			SharpIR( sensorCode _sensorType , uint8_t _sensorPin ) : sensorType( _sensorType ) , pin( _sensorPin ) {}

			int getDistance(bool avoidBurstRead = true);

			static sensorCode GP2Y0A41SK0F = 0 ;
			static sensorCode GP2Y0A21YK0F = 1 ;
			static sensorCode GP2Y0A02YK0F = 3 ;

		protected:

			uint8_t sensorType, pin ;

		private:

			uint32_t lastTime = 0 ;
	};

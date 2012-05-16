

#ifndef __VOLTAGE_H
#define __VOLTAGE_H

#include "../common/global.h"

enum {
	VOLTAGE_OS_0;		//	take a single reading
	VOLTAGE_OS_1;		//	take four samples and average
	VOLTAGE_OS_2;		//	take 16 samples and average
	VOLTAGE_OS_3;		//	take 32 samples and average
	VOLTAGE_OS_4;		//	take 64 samples and average
	VOLTAGE_OS_5;		//	take 128 samples and average
};
typedef uint8_t voltage_oversampling_t;

//
//	estimate the 5V bus voltage
//
// see: http://www.ikalogic.com/avr-monitor-power-supply-voltage-for-free/
//
float voltage_5V(voltage_oversampling_t oss);

//
//	estimate the 3.3V bus voltage
//
float voltage_3V(voltage_oversampling_t oss);


#endif
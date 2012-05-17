//////////////////////////////////////////////////////////////////////////////////////////
//	
//	File		: 'terminal.c'
//	Author		: Alan K. Duncan <duncan.alan@mac.com>
//	Created		: 2012-05-15
//	Revised		: 2012-05-17
//	Version		: 1.0
//	Target MCU	: ATmega644A
//	
//	This file provides voltage reporting functionality for the HAB project.
//
//////////////////////////////////////////////////////////////////////////////////////////

#ifndef __VOLTAGE_H
#define __VOLTAGE_H

#include "../common/global.h"

enum {
	VOLTAGE_OS_0,		//	take a single reading
	VOLTAGE_OS_1,		//	take four samples and average
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
//////////////////////////////////////////////////////////////////////////////////////////
//	
//	File		: 'hih4030.h'
//	Author		: Alan K. Duncan <duncan.alan@mac.com>
//	Created		: 2012-05-01
//	Revised		: 2012-05-09
//	Version		: 1.0
//	Target MCU	: ATmega644A
//	
//	This file provides an interface to the HIH4030 analog humidity sensor
//
//
//////////////////////////////////////////////////////////////////////////////////////////

#ifndef HIH4030_H_
#define HIH4030_H_

#include "../common/global.h"

#define HIH4030_ADC_CHANNEL 7

void hih4030_init(void);
u08 hih4030_sensor_rh(void);
u08 hih4030_compensated_rh(s16 temperature);



#endif /* HIH4030_H_ */
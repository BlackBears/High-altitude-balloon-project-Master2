//////////////////////////////////////////////////////////////////////////////////////////
//	
//	File		: 'hih4030.c'
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

#include "hih4030.h"
#include <util/delay.h>

static u08 r(double x) {
      if( x >= 0 )
         return (u08) (x+0.5f);
      return (u08) (x-0.5f);
   }

void hih4030_init(void) {
	a2dInit();
}

u08 hih4030_sensor_rh(void) {
	a2dInit();
	uint16_t rh_adc_value = a2dConvert10bit(HIH4030_ADC_CHANNEL);
	float v_ratio = rh_adc_value/1023.0f;
	float rh = (v_ratio - 0.16f)/0.0062f;
	return r(rh);
}

u08 hih4030_compensated_rh(s16 temperature) {
	u08 sensor_rh = hih4030_sensor_rh();
	float comp_rh = sensor_rh/(1.0546f - 0.00216f * temperature);
	return r(comp_rh);
}
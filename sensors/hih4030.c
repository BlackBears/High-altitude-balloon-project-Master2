/*
 * hih4030.c
 *
 * Created: 4/28/2012 11:18:05 PM
 *  Author: Administrator
 */ 

#include "hih4030.h"

u08 r(double x) {
      if (x >= 0)
         return (u08) (x+0.5);
      return (u08) (x-0.5);
   }

void hih4030_init(void) {
	a2dInit();
}

u08 hih4030_sensor_rh(void) {
	uint16_t rh_adc_value = a2dConvert10bit(HIH4030_ADC_CHANNEL);
	float v_ratio = rh_adc_value/1023.0;
	float rh = (v_ratio - 0.16)/0.0062;
	return r(rh);
}

u08 hih4030_compensated_rh(s16 temperature) {
	u08 sensor_rh = hih4030_sensor_rh();
	float comp_rh = sensor_rh/(1.0546 - 0.00216 * temperature);
	return r(comp_rh);
}
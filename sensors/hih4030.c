/*
 * hih4030.c
 *
 * Created: 4/28/2012 11:18:05 PM
 *  Author: Administrator
 */ 

#include "hih4030.h"
#include <util/delay.h>

u08 r(double x) {
      if (x >= 0)
         return (u08) (x+0.5);
      return (u08) (x-0.5);
   }

void hih4030_init(void) {
	a2dInit();
}

u08 hih4030_sensor_rh(void) {
	/*
	ADMUX &= ~0b00011111;	//	clear the ADC channel bits
	ADMUX |= HIH4030_ADC_CHANNEL;
	ADMUX = (1<<REFS0);		// Use Vcc as the analog reference;
	ADCSRA |= (1<<ADEN);	// turn on the ADC circuitry
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // prescaler /128
	for(u16 timeout = 0; timeout < 1000; timeout++) {
		if( (ADCSRA & (1<<ADIF)) ) 
			break;
		_delay_us(10);	//	delay 10 us for conversion
	}
	ADCSRA|=(1<<ADIF);
	uint16_t rh_adc_value = (ADC);
	*/
	a2dInit();
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
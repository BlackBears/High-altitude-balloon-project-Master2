

#include "voltage.h"
#include "../capabilities/a2d.h"

#include "../capabilities/uart2.h"
#include <string.h>

static float _voltage(BOOL is5V, voltage_oversampling_t oss) {
	a2dInit();
	a2dSetReference((is5V)?ADC_REFERENCE_AVCC:ADC_REFERENCE_AREF);
	a2dSetChannel((is5V)?ADC_CH_122V:ADC_CH_ADC6);
	
	float volt;
	uint16_t adc_data;
	if( oss == VOLTAGE_OS_0 ) {
		adc_data = a2dConvert10bit((is5V)?ADC_CH_122V:ADC_CH_ADC6);
	}	// no oversampling
	else {
		if( oss == VOLTAGE_OS_1 ) {
			uint16_t accumulator = 0;
			for(uint8_t i = 0; i < 4; i++) {
				accumulator += a2dConvert10bit((is5V)?ADC_CH_122V:ADC_CH_ADC6);
			}
			uint16_t adc_data = accumulator>>2;
		} // oversampling 1
		else {
			return 0;
		}	//	invalid oversampling value
	}
	if( is5V ) {
		volt = 1100.0f * (1023.0f/(float)adc_data);
	}
	else {
		volt = 5000.0f * (float)adc_data/1023.0f;
	}
	return volt;
}

//
//	estimate the 5V bus voltage
//
float voltage_5V(voltage_oversampling_t oss) {
	return _voltage(TRUE,oss);
}

float voltage_3V(voltage_oversampling_t oss) {
	return _voltage(FALSE,oss);
}



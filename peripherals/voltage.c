

#include "voltage.h"
#include "../capabilities/a2d.h"

static float _voltage(BOOL is5V, voltage_oversampling_t oss) {
	a2dInit();
	a2dSetReference((is5V)?ADC_REFERENCE_AVCC:ADC_REFERENCE_AREF);
	a2dSetChannel((is5V)?ADC_CH_122V:ADC_CH_ADC6);
	
	float volt;
	uint16_t adc_data;
	if( oss == VOLTAGE_OS_0 ) {
		adc_data = a2dConvert10bit(ADC_CH_122V);
	}
	else {
		uint8_t oss_quant;
		switch(oss) {
			case VOLTAGE_OS_1:
				oss_quant = 2;
				break;
			case VOLTAGE_OS_2:
				oss_quant = 4;
				break;
			case VOLTAGE_OS_3:
				oss_quant = 5;
				break;
			case VOLTAGE_OS_4:
				oss_quant = 6;
				break;
			case VOLTAGE_OS_5:
				oss_quant = 7;
				break;
		}
		uint16_t accumulator = 0;
		for(uint8_t i = 1; i < (1<<oss_quant); i++) {
			accumulator += a2dConvert10bit((is5V)?ADC_CH_122V:ADC_CH_ADC6);
		}
		uint16_t adc_data = (accumulator>>oss_quant);
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
	return _voltage(YES,oss);
}

float voltage_3V(voltage_oversampling_t oss) {
	return _voltage(NO,oss);
}





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
	}
	else {
		uint8_t oss_quant;
		switch(oss) {
			case VOLTAGE_OS_1:
				oss_quant = 4;
				break;
			case VOLTAGE_OS_2:
				oss_quant = 16;
				break;
			case VOLTAGE_OS_3:
				oss_quant = 32;
				break;
			case VOLTAGE_OS_4:
				oss_quant = 64;
				break;
		}
		uint16_t accumulator = 0;
		for(uint8_t i = 0; i < oss_quant; i++) {
			accumulator += a2dConvert10bit((is5V)?ADC_CH_122V:ADC_CH_ADC6);
		}
		uint16_t adc_data = accumulator/oss_quant;
		//char b[25];
		//sprintf(b,"oss %02d|acc = %03d\r",oss_quant,accumulator);
		//uartSendString(1,b);
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



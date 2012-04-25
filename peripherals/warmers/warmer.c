

#include "warmer.h"
#include "../../capabilities/a2d.h"
#include <avr/io.h>

void warmer_init(void) {
	a2dInit();
}

void warmer_read_temp( volatile warmer_t *warmer) {
	warmer->current_temp = a2dConvert10bit(warmer->adc_channel)/2;
}
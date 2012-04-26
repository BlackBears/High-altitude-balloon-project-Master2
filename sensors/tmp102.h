/*
 * tmp102.h
 *
 * Created: 4/13/2012 11:14:38 PM
 *  Author: Administrator
 */ 


#ifndef TMP102_H_
#define TMP102_H_

#include "../common/global.h"

#define TMP102_INVALID_TEMP	0x0FFF

enum {
	k_tmp102_addr0_gnd = 0x90,
	k_tmp102_addr0_vcc = 0x92,
	k_tmp102_addr0_sda = 0x94,
	k_tmp102_addr0_scl = 0x96
};
typedef u08 tmp102_addr_t;

typedef struct {
	tmp102_addr_t address;
	s16 temperature;
	BOOL is_valid;
} tmp102_t;

void tmp102_read_temp(volatile tmp102_t *device);

#endif /* TMP102_H_ */
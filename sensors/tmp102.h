/*
 * tmp102.h
 *
 * Created: 4/13/2012 11:14:38 PM
 *  Author: Administrator
 */ 


#ifndef TMP102_H_
#define TMP102_H_

#include "../common/global.h"

enum {
	k_tmp102_addr0_gnd = 0x90,
	k_tmp102_addr0_vcc = 0x92,
	k_tmp102_addr0_sda = 0x94,
	k_tmp102_addr0_scl = 0x96
};
typedef u08 tmp102_addr_t;

s16 tmp102_read_temp(tmp102_addr_t address);

#endif /* TMP102_H_ */
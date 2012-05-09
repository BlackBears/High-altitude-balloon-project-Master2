//////////////////////////////////////////////////////////////////////////////////////////
//	
//	File		: 'tmp102.h'
//	Author		: Alan K. Duncan <duncan.alan@mac.com>
//	Created		: 2012-05-01
//	Revised		: 2012-05-09
//	Version		: 1.0
//	Target MCU	: ATmega644A
//	
//	This file provides an interface to the TMP102 temperature sensor via I2C 
//	interface.
//
//
//////////////////////////////////////////////////////////////////////////////////////////

#ifndef TMP102_H_
#define TMP102_H_

#include "../common/global.h"
#include "../common/states.h"

#define TMP102_INVALID_TEMP	0x0FFF

#define TMP102_ADDR_GND	0x90
#define TMP102_ADDR_VCC 0x92
#define TMP102_ADDR_SDA	0x94
#define TMP102_ADDR_SCL	0x96
typedef u08 tmp102_addr_t;

#define TMP102_LOC_INT	0x00
#define TMP102_LOC_EXT	0x01
typedef u08 tmp102_location_t;

typedef struct {
	tmp102_addr_t address;
	tmp102_location_t location;
	volatile s16 temperature;
	volatile BOOL is_valid;
	sensor_status_t status;
} tmp102_t;

void tmp102_set_pwr(tmp102_t *device, BOOL pwr_status);
void tmp102_read_temp(tmp102_t *device);

#endif /* TMP102_H_ */
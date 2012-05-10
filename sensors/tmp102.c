//////////////////////////////////////////////////////////////////////////////////////////
//	
//	File		: 'tmp102.c'
//	Author		: Alan K. Duncan <duncan.alan@mac.com>
//	Created		: 2012-05-01
//	Revised		: 2012-05-09
//	Version		: 1.0
//	Target MCU	: ATmega644A
//	
//	This file provides an interface to the TMP102 temperature sensor via I2C 
//	interface.
//	
//	Some of the implementation of the temperature conversion code is from Sparkfun's
//	C example on their website.
//
//
//////////////////////////////////////////////////////////////////////////////////////////

#include "tmp102.h"
#include "../common/global.h"
#include "../common/pindefs.h"
#include "../capabilities/i2c.h"
#include "../capabilities/uart2.h"
#include <util/delay.h>

/*	DEVICE BASE ADDRESS */
#define TMP102_BASE_ADDRESS 0x90	//	assumes ADR0 is tied to GND

/*	REGISTER ADDRESSES */

#define TMP102_TEMP_REG		0x00
#define TMP102_CONFIG_REG	0x01
#define TMP102_LOW_REG		0x02
#define TMP102_HIGH_REG		0x03



void tmp102_set_pwr(tmp102_t *device, BOOL pwr_status) {
    if( device->location == TMP102_LOC_INT ) {
        DDR(INT_TEMP_PWR_PORT) |= (1<<INT_TEMP_PWR_PIN);
        if( pwr_status )
            INT_TEMP_PWR_PORT |= (1<<INT_TEMP_PWR_PIN);
        else
            INT_TEMP_PWR_PORT &= ~(1<<INT_TEMP_PWR_PIN);
    }
    else {
        DDR(EXT_TEMP_PWR_PORT) |= (1<<EXT_TEMP_PWR_PIN);
        if( pwr_status )
            EXT_TEMP_PWR_PORT |= (1<<EXT_TEMP_PWR_PIN);
        else
            EXT_TEMP_PWR_PORT &= ~(1<<EXT_TEMP_PWR_PIN);
    }
    device->status.power = (pwr_status)?PWR_ON:PWR_OFF;
}

#define TMP102_USE_I2C_INTERRUPT 0
void tmp102_read_temp(tmp102_t *device) {
	u08 data[2];
	data[0] = TMP102_TEMP_REG;
	//sprintf(buffer,"DEVICE ADDR = %02X | DEVICE LOC = %02X\r",device->address,device->location);
	//uart1_puts(buffer);
#if TMP102_USE_I2C_INTERRUPT == 0
	cli();	//	disable interrupts to prevent corruption of data
	u08 retval = i2cMasterSendNI(device->address,1,data);
	if( retval != I2C_OK ) { 
		device->is_valid = FALSE;
		device->temperature = TMP102_INVALID_TEMP;
		//uart1_puts("*** ERROR INVALID TEMP ***\r");
		sei();
		return;
	}
	_delay_ms(5);
	retval = i2cMasterReceiveNI(device->address,2,data);
	if( retval != I2C_OK ) { 
		device->is_valid = FALSE;
		device->temperature = TMP102_INVALID_TEMP;
		sei();
		return;
	}
	sei();	//	re-enable interrupts
#else
	i2cMasterSend(device->address,1,data);
	_delay_ms(5);
	i2cMasterReceive(device->address,1,data);
#endif
	
	device->is_valid = TRUE;
	u08 msb,lsb;
	msb = data[0];
	lsb = data[1];
	s16 temp = (msb<<8) | lsb;
	temp >>= 4; //The TMP102 temperature registers are left justified, correctly right justify them
	
	//The tmp102 does twos compliment but has the negative bit in the wrong spot, so test for it and correct if needed
	if(temp & (1<<11))
		temp |= 0xF800; //Set bits 11 to 15 to 1s to get this reading into real twos compliment

	//printf("%02d\n", temp);

	//But if we want, we can convert this directly to a celsius temp reading
	//temp *= 0.0625; //This is the same as a divide by 16
	//temp >>= 4; //Which is really just a shift of 4 so it's much faster and doesn't require floating point
	//Shifts may not work with signed ints (negative temperatures). Let's do a divide instead
	temp /= 16;

	device->temperature = temp;
}

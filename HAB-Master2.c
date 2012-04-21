/*
 * HAB_Master2.c
 *
 * Created: 4/13/2012 10:52:52 PM
 *  Author: Administrator
 */ 


#include <avr/io.h>
#include "capabilities/i2c.h"
#include "peripherals/ds1307.h"
#include "peripherals/bmp085.h"
#include "peripherals/tmp102.h"
#include "peripherals/ds18x20.h"
#include "capabilities/fat/diskio.h"
#include "common/global.h"
#include "common/types.h"
#include "common/pindefs.h"
#include "common/states.h"

flight_status_t flight_status;

/*	FUNCTION PROTOTYPES	*/

void init(void);

int main(void)
{
	global_states.system_state = k_system_state_configure;
	global_states.flight_state = k_state_preflight;
	
	i2cInit();							//	init I2C bus
    while(1)
    {
        //TODO:: Please write your application code 
    }
}

void init(void) {
	//	start out in configuration state
	global_states.system_state = k_system_state_configure;
	//	our flight state is preflight
	global_states.flight_state = k_state_preflight;
	
	i2cInit();							//	init I2C bus
	
	//	initialize the RTC and the 1Hz pulse
	ds1307_init(kDS1307Mode24HR);		//	init RTC
	ds1307_sqw_set_mode(k_ds1307_sqw_mode_a);
	ds1307_sqw_enable();
	
	//	1Hz square wave pulse interrupt triggers on falling edge
	#if RTC_1HZ_INT == 7
		EICRB &= ~(1<<ISC50);
		EICRB &= ~(1<<ISC51);
		EIMSK |= (1<<INT7);
		sei();
	#endif
}

/*	EXTERNAL INTERRUPTS	*/
ISR(INT7_vect) {
	//	this is our 1Hz interrupt
}

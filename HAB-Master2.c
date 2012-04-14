/*
 * HAB_Master2.c
 *
 * Created: 4/13/2012 10:52:52 PM
 *  Author: Administrator
 */ 


#include <avr/io.h>
#include "capabilities/i2c.h"
#include "peripherals/ds1307.h"
#include "common/global.h"
#include "common/types.h"

global_states_t global_states;

int main(void)
{
	global_states.system_state = k_system_state_configure;
	global_states.flight_state = k_state_preflight;
	
	i2cInit();							//	init I2C bus
	ds1307_init(kDS1307Mode24HR);		//	init RTC
    while(1)
    {
        //TODO:: Please write your application code 
    }
}
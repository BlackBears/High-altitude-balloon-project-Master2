/*
 * HAB_Master2.c
 *
 * Created: 4/13/2012 10:52:52 PM
 *  Author: Administrator
 */ 

#include "common/global.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include "capabilities/i2c.h"
#include "peripherals/openlog.h"
#include "peripherals/ds1307.h"
#include "peripherals/warmers/warmer.h"
#include "peripherals/warmers/warmer_output.h"
#include "peripherals/warmers/warmer_timing.h"
#include "sensors/bmp085.h"
#include "sensors/tmp102.h"
#include "common/types.h"
#include "common/pindefs.h"
#include "common/states.h"
#include "capabilities/vfd.h"

flight_status_t flight_status;
char buffer[20];
warmer_t battery_warmer;
volatile tmp102_t internal_temperature;
volatile tmp102_t external_temperature;

/*	FUNCTION PROTOTYPES	*/

void init(void);
void _init_rtc(void);

int main(void)
{
	i2cInit();
	_delay_ms(10);
	vfd_init();
	bmp085_init();
	
	_init_rtc();
	warmer_init();
	
	battery_warmer.adc_channel = 4;
	battery_warmer.min_temp = 10;
	battery_warmer.max_temp = 15;
	battery_warmer.type = k_warmer_battery;
	
	_delay_ms(1000);
	vfd_cls();
	sprintf(buffer,"Welcome to HAB!");
	vfd_puts(buffer);
	_delay_ms(1000);
	

	vfd_cls();
	/*
	sprintf(buffer,"Initializing Open Log");
	vfd_puts(buffer);
	open_log_init();
	_delay_ms(1000);
	vfd_cr();
	sprintf(buffer,"Powering Open Log");
	vfd_puts(buffer);
	open_log_set_pwr(TRUE);
	_delay_ms(1000);
	vfd_cls();
	sprintf(buffer,"Creating test file.");
	vfd_puts(buffer);
	open_log_write_test();
	_delay_ms(2000);
	*/
	internal_temperature.address = k_tmp102_addr0_gnd;
	external_temperature.address = k_tmp102_addr0_vcc;
	
    while(1)
    {
		u08 hour = ds1307_hours();
		u08 minute = ds1307_minutes();
		u08 second = ds1307_seconds();
		sprintf(buffer,"%02d:%02d:%02d",hour,minute,second);
		vfd_cls();
		vfd_puts(buffer);

		tmp102_read_temp(&internal_temperature);
		tmp102_read_temp(&external_temperature);
		vfd_cr();
		//s16 tmp1 = tmp102_read_temp(k_tmp102_addr0_gnd);
		//s16 ext_temp = tmp102_read_temp(k_tmp102_addr0_vcc);
		if( internal_temperature.is_valid ) {
			sprintf(buffer,"TI = %02dC",internal_temperature.temperature);
			vfd_puts(buffer);
		}
		else {
			sprintf(buffer,"TI ABSENT");
			vfd_puts(buffer);
		}
		if( external_temperature.is_valid ) {
			sprintf(buffer," TE = %02dC",external_temperature.temperature);
			vfd_puts(buffer);
		}
		else {
			sprintf(buffer," TE ABSENT");
			vfd_puts(buffer);
		}
		//vfd_cls(); sprintf(buffer,"Read temps"); vfd_puts(buffer);
		
		//sprintf(buffer,"TMP1 = %02dC EXT = %02dC",tmp1,ext_temp);
		//vfd_puts(buffer);
		_delay_ms(2000);
		vfd_cls();
		
		long bp, bt;
		bmp085_convert(&bt,&bp);
		sprintf(buffer,"P = %ld Pa",bp);
		vfd_puts(buffer);
		vfd_cr();
		sprintf(buffer,"T = %ld x10C",bt);
		vfd_puts(buffer);
		_delay_ms(1000);
		vfd_cls();
		
		warmer_read_temp(&battery_warmer);
		sprintf(buffer,"BAT = %02d C",battery_warmer.current_temp);
		vfd_puts(buffer);
		_delay_ms(1000);
		vfd_cls();
    }
}

/*	INITIALIZATION	*/
void _init_rtc(void) {
	ds1307_init(kDS1307Mode24HR);
	ds1307_sqw_set_mode(k_ds1307_sqw_mode_a);
	
	//ds1307_set_hours(06);
	//ds1307_set_minutes(11);
	//ds1307_set_seconds(40);
	DDRD |= (1<<PD7);

	//PORTE &= ~(1<<PE7);
	//EICRB &= ~(1<<ISC70);
	//EICRB &= ~(1<<ISC71);
	//EIMSK |= (1<<INT7);
	//sei();
}

void _init_warmers(void) {
    warmer_controller_init();   // init a2d and pid params for warmers
    warmer_setup();             // initiate warmer(s), defined in warmer_output.h
    warmer_timing_setup();      // setup 64Hz interrupt on TIMER1, def in warmer_timing.h
}

/*	EXTERNAL INTERRUPTS	*/
ISR(INT7_vect) {
	//	this is our 1Hz interrupt
	PORTD ^= (1<<PD7);
}

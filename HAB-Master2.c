/*
 * HAB_Master2.c
 *
 * Created: 4/13/2012 10:52:52 PM
 *  Author: Administrator
 */ 

#define BAUD 9600

#include "common/global.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <stdio.h>
#include <stdlib.h>
#include "capabilities/i2c.h"
#include "capabilities/uart.h"
#include "peripherals/openlog.h"
#include "peripherals/ds1307.h"
#include "peripherals/warmers/warmer.h"
#include "peripherals/warmers/warmer_output.h"
#include "peripherals/warmers/warmer_timing.h"
#include "peripherals/mux.h"
#include "sensors/bmp085.h"
#include "sensors/tmp102.h"
#include "common/types.h"
#include "common/pindefs.h"
#include "common/states.h"
#include "capabilities/vfd.h"

flight_status_t flight_status;
time_t rtc;
char buffer[20];
u08 last_second;                        //  used to track 1Hz tasks
warmer_t battery_warmer;
volatile tmp102_t internal_temperature;
volatile tmp102_t external_temperature;
volatile bmp085_t bmp085;



/*	FUNCTION PROTOTYPES	*/

void init(void);
void _init_rtc(void);
void _init_bmp085(void);
void _init_tmp102(void);

void read_rtc(void);
void read_sensors(void);

int main(void)
{
    last_second = 0;
    
	i2cInit();          //  initialize the I2C bus
	_delay_ms(10);      //  wait until stabilizes
	vfd_init();         //  init our display
	bmp085_init();      //  init the barometric pressure sensor
	
	mux_init();         //  setup UART1 & set terminal as output
	flight_status.serial_channel = k_serial_out_terminal;
	u16 baud_rate = (UBRRH_VALUE << 8) | UBRRL_VALUE;
	uart_init(baud_rate);
	uart1_init(baud_rate);
	
	_init_rtc();        //  initialize our clock
	warmer_init();      //  initialize the warmers
	
	battery_warmer.adc_channel = 4;
	battery_warmer.min_temp    = 10;
	battery_warmer.max_temp    = 15;
	battery_warmer.type = k_warmer_battery;
	
	_delay_ms(1000);
	vfd_cls();
	sprintf(buffer,"Welcome to HAB!");
	vfd_puts(buffer);
	_delay_ms(1000);
	

	vfd_cls();
    while(1)
    {
        read_rtc();
        if( rtc.second != last_second ) {
            last_second = rtc.second;       //  reset our last_second
            
            //  do 1 Hz processing here
            read_sensors();
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

void _init_bmp085(void) {
    bmp085_pwr_set(&bmp085, TRUE);  //  power on
    _delay_ms(20);                  //  wait to stabilize a bit
    bmp085_init(&bmp085);           //  initialize
    
    //  if unable to read calibration values, note error status for flight
    if( !bmp085.is_valid ) {
        bmp085.status.connect_attempts++;
    }
    else {
        bmp085.status.status = k_peripheral_status_ok;  //  pressure monitor = OK
    }
}

void _init_tmp102(void) {
    //  setup our internal and external temp sensors
    internal_temperature.address = k_tmp102_addr0_gnd;
	internal_temperature.location = k_tmp102_loc_internal;
	external_temperature.address = k_tmp102_addr0_vcc;
	external_temperature.location = k_tmp102_loc_external;
	
	//  power them up
    tmp102_set_pwr(&internal_temperature);
    tmp102_set_pwr(&external_temperature);
    
    //  set our sensor status
    internal_temperature.connect_attempts = 0;
    external_temperature.connect_attempts = 0;
}

/*  READ SENSORS */

void read_sensors(void) {
    //  read internal temperature, if invalid, keep trying for 30s
    //  after 30s power it down to conserve power
    if( internal_temperature.status.power ) {
        tmp102_read_temp(&internal_temperature);    // read int temperature
        if( !internal_temperature.is_valid ) {
            u08 attempts = internal_temperature.status.connect_attempts;
            attempts++;
            internal_temperature.status.connect_attempts = attempts;
            if( attempts >= 30 ) {
                tmp102_set_pwr(&internal_temperature,FALSE);    // power down
                internal_temperature.status.status = k_peripheral_status_error;
            }
            uart1_puts("INT TEMP NA");
        }
        else {
            internal_temperature.status.status = k_peripheral_status_ok;
            internal_temperature.status.connect_attempts = 0;
        }
    }
    
    //  read external temperature.  if invalid, keep trying for 30s
    //  after 30s, power it down to converse power
    if( external_temperaturel.status.power ) {
        tmp102_read_temp(&external_temperature);    // read ext temperature
        if( !external_temperature.is_valid ) {
            u08 attempts = external_temperature.status.connect_attempts;
            attempts++;
            if( attempts >= 30 ) {
                tmp102_set_pwr(&external_temperature,FALSE);    // power down
                external_temperature.status.status = k_peripheral_status_error;
            }
            uart1_puts("EXT TEMP NA");
        }
        else {
            external_temperature.status.status = k_peripheral_status_ok;
            external_temperature.status.connect_attempts = 0;
        }
    }
    
    //  read barometric pressure if the sensor is powered
    if( bmp085.status.power ) {
        bmp085_convert(&bmp085);                    // read pressure
        if( !bmp085.is_valid ) {
            u08 attempts = bmp085.status.connect_attempts;
            attempts++;
            if( attempts >= 30 ) {
                //  power down the barometric pressure sensor
                //  if connect attempts exceed threshold
                bmp085_set_pwr(&bmp085,FALSE);
                bmp085.status.status = k_peripheral_status_error;
            }
        }
         else {
            bmp085.status.status = k_peripheral_status_ok;
            bmp085.status.connect_attempts = 0;
        }
    }		    
}
}

/*  TIME    */

void read_rtc(void) {
    rtc.hour = ds1307_hours();
    rtc.minute = ds1307_minutes();
    rtc.second = ds1307_seconds();
}

/*  REPORTING */

//
//  send environmental data to whichever UART1 vector is active
//  
void report_enviro {
    //  format report differently depending on output vector
    if( flight_status.serial_channel != k_serial_out_lcd ) {
        sprintf(buffer,"$ENV%02d%02d%02d",rtc.hour,rtc.minute.rtc.second);
        uart1_puts(buffer);
        if( internal_temperature.status.status == k_peripheral_status_ok ) {
            sprintf(buffer,"IT%02d",internal_temperature.temperature);
            uart1_puts(buffer);
        }
        else {
            uart1_puts("ITNA");
        }
        if( external_temperature.status.status = k_peripheral_status_ok ) {
            sprintf(buffer,"ET%02d",external_temperature.temperature);
            uart1_puts(buffer);
        }
        else {
            uart1_puts("ETNA");
        }
        if( bmp085.status.status = k_peripheral_status_ok ) {
            sprintf(buffer,"BP%06d",bmp085.pressure);
            uart1_puts(buffer);
        }
        else {
            uart1_puts("BPNA");
        }
        uart1_puts("\r");
    }   //  terminal or OpenLog destination
    else {
        //
    }
}

/*	EXTERNAL INTERRUPTS	*/
ISR(INT7_vect) {
	//	this is our 1Hz interrupt
	PORTD ^= (1<<PD7);
}

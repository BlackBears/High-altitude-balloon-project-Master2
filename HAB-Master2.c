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
#include <avr/wdt.h>
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

/************************************************************************/
/* GLOBAL VARIABLES                                                     */
/************************************************************************/
flight_status_t flight_status;
time_t rtc;
char buffer[60];
u08 last_second;                        //  used to track 1Hz tasks
warmer_t battery_warmer;
tmp102_t internal_temperature;
tmp102_t external_temperature;
bmp085_t bmp085;
long temperature, pressure;


/************************************************************************/
/* FUNCTION PROTOTYPES                                                  */
/************************************************************************/

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
	
	mux_init();         //  setup UART1 & set terminal as output
	flight_status.serial_channel = k_serial_out_terminal;
	u16 baud_rate = (UBRRH_VALUE << 8) | UBRRL_VALUE;
	uart_init(baud_rate);
	uart1_init(baud_rate);
	
	//	show welcome message
	uart1_putc(0x0C);	//	?cls
	sprintf(buffer,"Welcome to HAB!\r");
	uart1_puts(buffer);

	/*////////////////////////////////////////////////////////////////////////
	/	I2C bus initialization
	/	Note that for unclear reasons, the BMP085 must be initialized first
	/	followed by the TMP102 sensors.
	/////////////////////////////////////////////////////////////////////////*/
	i2cInit();          //  initialize the I2C bus
	_delay_ms(10);      //  wait until stabilizes
	_init_bmp085();     //  init the barometric pressure sensor
	_delay_ms(5);
	_init_tmp102();		//	init the temperature monitors
	_delay_ms(5);
	_init_rtc();		//  initialize our clock
	warmer_controller_init();      //  initialize the warmers
	
	battery_warmer.adc_channel = 4;
	battery_warmer.min_temp    = 10;
	battery_warmer.max_temp    = 15;
	battery_warmer.type = k_warmer_battery;
	
	_delay_ms(100);
	
	_delay_ms(1000);
	
    while(1)
    {
		if( rtc.new_second ) {
			rtc.new_second = FALSE;
            last_second = rtc.second;       //  reset our last_second
            
            //  do 1 Hz processing here
            read_sensors();
			report_enviro();
        }
		//sprintf(buffer,"%02d:%02d:%02d\r",rtc.hour,rtc.minute,rtc.second);
		//uart1_puts(buffer);
		_delay_ms(200);
	}			
}

/************************************************************************/
/* INITIALIZATION                                                       */
/************************************************************************/

void _init_watchdog_timer(void) {
	cli();
	wdt_reset();
	WDTCR |= (1<<WDP2) | (1<<WDP1);
	WDTCR |= (1<<WDE) | (1<<WDCE);
	sei();
}
void _init_rtc(void) {
	ds1307_init(kDS1307Mode24HR);
	ds1307_sqw_set_mode(k_ds1307_sqw_mode_a);
	
	//ds1307_set_hours(06);
	//ds1307_set_minutes(11);
	//ds1307_set_seconds(40);
#if RTC_1HZ_INT == 7
	PORTE &= ~(1<<PE7);
	EICRB &= ~(1<<ISC70);
	EICRB &= ~(1<<ISC71);
	EIMSK |= (1<<INT7);
	sei();
#endif
}

void _init_warmers(void) {
    warmer_controller_init();   // init a2d and pid params for warmers
    warmer_setup();             // initiate warmer(s), defined in warmer_output.h
    warmer_timing_setup();      // setup 64Hz interrupt on TIMER1, def in warmer_timing.h
}

void _init_bmp085(void) {
	uart1_puts("Will init bmp085 from main\r");
    bmp085_pwr_set(&bmp085, TRUE);  //  power on
    _delay_ms(20);                  //  wait to stabilize a bit
	uart1_puts("Will call bmp085 initialization\r");
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
    internal_temperature.address = TMP102_ADDR_GND;
	internal_temperature.location = TMP102_LOC_INT;
	external_temperature.address = TMP102_ADDR_VCC;
	external_temperature.location = TMP102_LOC_EXT;
	//sprintf(buffer,"main: ET ADDR = %02X\r",external_temperature.address);
	//uart1_puts(buffer);
	
	//  power them up
    tmp102_set_pwr(&internal_temperature,TRUE);
	_delay_ms(20);
    tmp102_set_pwr(&external_temperature,TRUE);
    
    //  set our sensor status
    internal_temperature.status.connect_attempts = 0;
    external_temperature.status.connect_attempts = 0;
}

/*  READ SENSORS */

#define FAULT_TOLERANT 1
#define NOT_FAULT_TOLERANT 0
#define FAULT_TOLERANCE_MODE NOT_FAULT_TOLERANT

void read_sensors(void) {
	tmp102_read_temp(&external_temperature);    // read ext temperature
	tmp102_read_temp(&internal_temperature);
	
	bmp085Convert(&temperature, &pressure);
	
	/*
#if FAULT_TOLERANCE_MODE == FAULT_TOLERANT
    //  read internal temperature, if invalid, keep trying for 30s
    //  after 30s power it down to conserve power
    if( internal_temperature.status.power ) {
        tmp102_read_temp(&internal_temperature);    // read int temperature
		//sprintf(buffer,"IT = %02dC\r",internal_temperature.temperature);
		//uart1_puts(buffer);
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
	else {
		uart1_puts("IT is not powered\r");	
	}		
#else
	tmp102_read_temp(&internal_temperature);
#endif
    _delay_ms(20);
	
#if FAULT_TOLERANCE_MODE == FAULT_TOLERANT
    //  read external temperature.  if invalid, keep trying for 30s
    //  after 30s, power it down to converse power
    if( external_temperature.status.power ) {
        tmp102_read_temp(&external_temperature);    // read ext temperature
		//sprintf(buffer,"ET = %02dC\r",external_temperature.temperature);
		//uart1_puts(buffer);
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
	else {
		uart1_puts("ET is not powered\r");	
	}		
#else
	tmp102_read_temp(&external_temperature);
#endif
    _delay_ms(20);
    //  read barometric pressure if the sensor is powered
    if( bmp085.status.power ) {
        bmp085_convert(&bmp085);                    // read pressure
        if( !bmp085.is_valid ) {
            u08 attempts = bmp085.status.connect_attempts;
            attempts++;
            if( attempts >= 30 ) {
                //  power down the barometric pressure sensor
                //  if connect attempts exceed threshold
                bmp085_pwr_set(&bmp085,FALSE);
                bmp085.status.status = k_peripheral_status_error;
            }
        }
         else {
            bmp085.status.status = k_peripheral_status_ok;
            bmp085.status.connect_attempts = 0;
        }
    }	
	else {
		uart1_puts("BMP085 is not powered\r");
	}	    
	*/
}

/************************************************************************/
/* TIMEKEEPING                                                          */
/************************************************************************/
void read_rtc(void) {
    rtc.hour = ds1307_hours();
    rtc.minute = ds1307_minutes();
    rtc.second = ds1307_seconds();
}

/************************************************************************/
/* REPORTING                                                            */
/************************************************************************/

//
//  send environmental data to whichever UART1 vector is active
//  
void report_enviro(void) {
    //  format report differently depending on output vector
    if( flight_status.serial_channel != k_serial_out_lcd ) {
        sprintf(buffer,"$ENV%02d%02d%02d",rtc.hour,rtc.minute,rtc.second);
        uart1_puts(buffer);
		
		sprintf(buffer,"IT%02d",internal_temperature.temperature);
        uart1_puts(buffer);
		
		sprintf(buffer,"ET%02d",external_temperature.temperature);
        uart1_puts(buffer);
		
		sprintf(buffer,"BP%ldBPT%ld",pressure,temperature);
        uart1_puts(buffer);
			
		/*
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
            sprintf(buffer,"BP%06ldBPT%06ld",bmp085.pressure,bmp085.temperature);
            uart1_puts(buffer);
        }
        else {
            uart1_puts("BPNA");
        }
		*/
        uart1_puts("\r");
    }   //  terminal or OpenLog destination
    else {
        //
    }
}

/************************************************************************/
/* INTERRUPTS															*/
/************************************************************************/
ISR(INT7_vect) {
	read_rtc();
	rtc.new_second = TRUE;
}

ISR(RESET_vect) {
	
	
}
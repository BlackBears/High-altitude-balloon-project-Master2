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
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <stdio.h>
#include <stdlib.h>
#include "capabilities/i2c.h"
#include "capabilities/uart-644a.h"
#include "peripherals/openlog.h"
#include "peripherals/ds1307.h"
#include "peripherals/warmers/warmer.h"
#include "peripherals/warmers/warmer_output.h"
#include "peripherals/warmers/warmer_timing.h"
#include "peripherals/mux.h"
#include "peripherals/dx.h"
#include "peripherals/terminal.h"
#include "sensors/bmp085.h"
#include "sensors/tmp102.h"
#include "sensors/hih4030.h"
#include "common/types.h"
#include "common/pindefs.h"
#include "common/states.h"
//#include "capabilities/vfd.h"

/************************************************************************/
/* GLOBAL VARIABLES                                                     */
/************************************************************************/
flight_status_t flight_status;
time_t rtc;
char buffer[60];
warmer_t battery_warmer;
tmp102_t internal_temperature;
tmp102_t external_temperature;
bmp085_t bmp085;
long temperature, pressure;
u08 humidity;
u08 cdiv;
uint32_t rtc_millis;
u08 last_second;
uint32_t warmer_64Hz_millis;
uint8_t warmer_8Hz_div;

#define clockCyclesPerMicrosecond() ( F_CPU / 1600000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

// the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))

// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
static unsigned char timer0_fract = 0;


/************************************************************************/
/* FUNCTION PROTOTYPES                                                  */
/************************************************************************/

void init(void);
void _init_rtc(void);
void _init_bmp085(void);
void _init_tmp102(void);
void _init_timer0(void);

s16 get_internal_temperature();
s16 get_external_temperature();
long barometric_pressure();
u08 get_humidity();
void read_rtc(void);
void read_sensors(void);
void report_enviro(void);

BOOL internal_temperature_power();
BOOL external_temperature_power();
void set_internal_temperature_power(BOOL status);
void set_external_temperature_power(BOOL status);
void rtc_read_time(time_t *time);
void rtc_set_time(time_t *time);

unsigned long millis();

int main(void)
{
	DDRA |= (1<<PA0);
	PORTA &= ~(1<<PA0);
	mux_init();         //  setup UART1 & set terminal as output
	uart_init(9600);
	uart1_init(9600);
	sei();
	
	
    //wdt_disable();
	//wdt_enable(WDTO_4S);
	
	DO_AND_WAIT(_init_timer0(),10);
	
	rtc_millis = 0;
	warmer_64Hz_millis = 0;
	
	i2cInit();          //  initialize the I2C bus
	
	dx_indicator_init();
	
	flight_status.serial_channel = k_serial_out_terminal;
	flight_status.terminal_input.state = TERMINAL_WAITING;
	flight_status.terminal_input.timeout = millis() + 5000;		//	five seconds to respond
	terminal_init();
	read_rtc();
	sprintf(buffer,"%02d:%02d:%02d",rtc.hour,rtc.minute,rtc.second); uart1_puts(buffer);
	/*////////////////////////////////////////////////////////////////////////
	/	I2C bus initialization
	/	Note that for unclear reasons, the BMP085 must be initialized first
	/	followed by the TMP102 sensors.
	/////////////////////////////////////////////////////////////////////////*/
	_delay_ms(10);				   //  wait until stabilizes
	
	DO_AND_WAIT(_init_bmp085(),5);	//  init the barometric pressure sensor
	DO_AND_WAIT(_init_tmp102(),5);	//	init the temperature monitors
	DO_AND_WAIT(_init_rtc(),5);		//	init the real-time clock
	//warmer_controller_init();      //  initialize the warmers
	//warmer_setup();				   //  setup the warmer output
	//hih4030_init();
	//PORTA &= ~(1<<PA0);
    while(1)
    {
		uint32_t m = millis();
		if( flight_status.terminal_input.state == TERMINAL_WAITING ) {
			if( m >= flight_status.terminal_input.timeout ) {
				//	terminal did not register within timeout, so we will begin regular procedures
				flight_status.terminal_input.state = TERMINAL_OFF;
				uart1_puts_P("\rTerminal timed out\r");
				uart1_puts_P("Bye\r");
				//  redirect logging to the OpenLog
				mux_select_channel(MUX_OPEN_LOG);
				
			}
			else {
				//	we're waiting for terminal input, so let's get a character
				u16 terminal_data = uart1_getc();
				if( ((terminal_data>>8) != UART_NO_DATA) && ((char)terminal_data != 0x00) ) {
					if( (char)terminal_data != 0x0D )
						uart1_putc((char)terminal_data);
					terminal_process_char( (char)terminal_data );
					flight_status.terminal_input.state = TERMINAL_SELECTED;
				}	//	valid data on terminal
			}	//	waiting for terminal inside timeout
		}	//	terminal waiting
		else if( flight_status.terminal_input.state == TERMINAL_OFF ) {
			if( m - rtc_millis > 1000 ) {
				//  do 1 Hz processing here
				read_rtc();
				if( rtc.second != last_second ) {
					read_sensors();
					report_enviro();
					last_second = rtc.second;
				}	//	last second not processed			
            
				rtc_millis = m;
			}	//	~ 1000 ms passed
			//wdt_reset();
			//	update our warmer output at 64 Hz (~15 ms)
			if( m - warmer_64Hz_millis > 16) {
				warmer_update_64Hz();       //  update the controller output at 64Hz
				//  execute control update every 8 steps (64 Hz/8 = 8 Hz)
				if( ++warmer_8Hz_div == 8) {
					warmer_update_8Hz();
					warmer_8Hz_div = 0;
				}	//	8 Hz update
				warmer_64Hz_millis = m;
			}	//	64 Hz update
			wdt_reset();
			dx_indicator_update(m);
		} // terminal is not waiting
		else {
			u16 terminal_data = uart1_getc();
			if( ((terminal_data>>8) != UART_NO_DATA) && ((char)terminal_data != 0x00) ) {
				terminal_process_char( (char)terminal_data );
				if( (char)terminal_data != 0x0D )
					uart1_putc((char)terminal_data);
			}	//	valid data on terminal
		}	//	terminal mode
		//wdt_reset();
	} //	main loop
}	// main

/************************************************************************/
/* INITIALIZATION                                                       */
/************************************************************************/

void _init_rtc(void) {
	ds1307_init(kDS1307Mode24HR);
	ds1307_sqw_set_mode(k_ds1307_sqw_mode_a);
}

void _init_warmers(void) {
    warmer_controller_init();   // init a2d and pid params for warmers
    warmer_setup();             // initiate warmer(s), defined in warmer_output.h
    warmer_timing_setup();      // setup 64Hz interrupt on TIMER1, def in warmer_timing.h
}

void _init_bmp085(void) {
	DO_AND_WAIT(bmp085_pwr_set(&bmp085, TRUE),20);	//	power on and stabilize
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
	DO_AND_WAIT(tmp102_set_pwr(&internal_temperature,TRUE),20);
    tmp102_set_pwr(&external_temperature,TRUE);
    
    //  set our sensor status
    internal_temperature.status.connect_attempts = 0;
    external_temperature.status.connect_attempts = 0;
}

/*  READ SENSORS */

s16 get_internal_temperature() {
    tmp102_read_temp(&internal_temperature);
    return internal_temperature.temperature;
}

s16 get_external_temperature() {
    tmp102_read_temp(&external_temperature);
    return external_temperature.temperature;
}

long barometric_pressure() {
    bmp085Convert(&temperature, &pressure);
    return pressure;
}

u08 get_humidity() {
    tmp102_read_temp(&external_temperature); 
    humidity = hih4030_compensated_rh(external_temperature.temperature);
    return humidity;
}


#define FAULT_TOLERANT 1
#define NOT_FAULT_TOLERANT 0
#define FAULT_TOLERANCE_MODE NOT_FAULT_TOLERANT

void read_sensors(void) {
	tmp102_read_temp(&external_temperature);    // read ext temperature
	tmp102_read_temp(&internal_temperature);
	
	bmp085Convert(&temperature, &pressure);
	
	//	every 15 seconds, measure and compute the temperature-compensated relative humidity 
	if( rtc.second % 15 == 0) {
		humidity = hih4030_compensated_rh(external_temperature.temperature);
	}
	
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
/*  SENSOR POWER                                                        *
/************************************************************************/

BOOL internal_temperature_power() {
    return internal_temperature.status.power;
}

BOOL external_temperature_power() {
    return external_temperature.status.power;
}

void set_internal_temperature_power(BOOL status) {
    tmp102_set_pwr(&internal_temperature,status);
}

void set_external_temperature_power(BOOL status) {
    tmp102_set_pwr(&external_temperature,status);
}


/************************************************************************/
/* TIMEKEEPING                                                          */
/************************************************************************/
void read_rtc(void) {
    rtc.hour = ds1307_hours();
    rtc.minute = ds1307_minutes();
    rtc.second = ds1307_seconds();
}

void rtc_read_time(time_t *time) {
	read_rtc();
	memcpy(time,&rtc,sizeof(time_t));
}

void rtc_set_time(time_t *time) {
	ds1307_set_hours(time->hour);
	ds1307_set_minutes(time->minute);
	ds1307_set_seconds(time->second);
}

void _init_timer0(void) {
	TIMSK0 |= (1<<OCIE0A);				//	enable TIMER0 COMP interrupt
	sei();								//	enable global interrupts
	
	TCCR0B |= (1<<CS01) | (1<<CS00);	//	prescaler @ /64
	TCCR0A |= (1<<WGM01);				//	CTC mode
	OCR0A = 0xFA;						//	this val is hand-tuned with 'scope
}

ISR(TIMER0_COMPA_vect)
{
	// copy these to local variables so they can be stored in registers
	// (volatile variables must be read from memory on every access)
	unsigned long m = timer0_millis;
	unsigned char f = timer0_fract;
	//PORTA ^= (1<<PA0);
	
	m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		m += 1;
	}

	timer0_fract = f;
	timer0_millis = m;
	timer0_overflow_count++;
}

unsigned long millis()
{
   unsigned long m;
   uint8_t oldSREG = SREG;

   // disable interrupts while we read timer0_millis or we might get an
   // inconsistent value (e.g. in the middle of a write to timer0_millis)
   cli();
   m = timer0_millis;
   SREG = oldSREG;

   return m;
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
		
		sprintf(buffer,"HUM%03d",humidity); uart1_puts(buffer);
			
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

/*	USER INTERACTION */

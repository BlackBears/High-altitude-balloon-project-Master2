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
#include <string.h>
#include "capabilities/i2c.h"
#include "capabilities/uart2.h"
#include "peripherals/openlog.h"
#include "peripherals/ds1307.h"
#include "peripherals/warmers/warmer.h"
#include "peripherals/warmers/warmer_output.h"
#include "peripherals/warmers/warmer_timing.h"
#include "peripherals/mux.h"
#include "peripherals/dx.h"
#include "peripherals/terminal.h"
#include "gps/gps.h"
#include "capabilities/nmea.h"
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
flight_status_t flight_status;		//	our main flight status structure
time_t rtc;							// 	real-time clock
warmer_t battery_warmer;			//	battery warmer structure
tmp102_t internal_temperature;		//	internal temperature sensor
tmp102_t external_temperature;		// 	external temperature sensor
bmp085_t bmp085;					//	barometric pressure sensor
static long temperature = 0;		//	temperature from BP sensor
static long pressure = 0;			//	barometric pressure in Pascals (Pa)
static u08 humidity = 0;			//	external humidity reading
static uint32_t rtc_millis = 0;		//	ms to rtc read timeout
static uint32_t sensor_millis = 0;	//	ms to sensor read timeout
static uint32_t warmer_64Hz_millis = 0;	//	ms until next 64 Hz timeout for warmer pulse
static uint8_t warmer_8Hz_div = 0;	//	8 Hz counter for warmer power update

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

static void init(void);
static void _init_rtc(void);
static void _init_bmp085(void);
static void _init_tmp102(void);
static void _init_timer0(void);

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
void set_serial_channel(mux_channel_t chan);
void set_ignore_serial_data(BOOL state);

unsigned long millis();

#define USING_WARMERS 1
#define FORCE_SERIAL_OUTPUT_TERMINAL 1

int main(void) {
	wdt_disable();			//	disable
	wdt_enable(WDTO_4S);	//	then re-enable the watchdog timer with 4 second interrupt
	
	DDRB |= (1<<PB1); PORTB &= ~(1<<PB1);
	for(u08 i = 0; i < 10; i++) {
		PORTB ^= (1<<PB1);	
		_delay_ms(100);
	}		
	//	init the openlog
	open_log_init();	
	open_log_reset_nack();
	
	gps_init();			//	init the UART0, gps info and NMEA processor
	
	//	deal with UART1 initialization
 	mux_init();				//	init the serial multiplexer on UART1
	uart1Init();			//	set up our multiplexed UART1 port
	uartSetBaudRate(1,9600);
	sei();
	uartSendByte(1,0x0C);	//	clear the terminal
	
	//	initialize our TIMER0 which counts milliseconds
	DO_AND_WAIT(_init_timer0(),10);
	
	
	i2cInit();          //  initialize the I2C bus
	
	dx_indicator_init();
	
	flight_status.serial_channel = MUX_TERMINAL;
	flight_status.terminal_input.state = TERMINAL_WAITING;
	flight_status.terminal_input.timeout = millis() + 5000;		//	five seconds to respond
	flight_status.event.gps_altitude_timeout = millis() + 10000;
	terminal_init();
	read_rtc();
	////////////////////////////////////////////////////////////////////////
	//	I2C bus initialization
	//	Note that for unclear reasons, the BMP085 must be initialized first
	//	followed by the TMP102 sensors.
	/////////////////////////////////////////////////////////////////////////
	_delay_ms(10);				    //  wait until stabilizes
	
	DO_AND_WAIT(_init_bmp085(),5);	//  init the barometric pressure sensor
	DO_AND_WAIT(_init_tmp102(),5);	//	init the temperature monitors
	DO_AND_WAIT(_init_rtc(),5);		//	init the real-time clock
	DO_AND_WAIT(_init_warmers(),2);	//	init the warmers
	hih4030_init();					//	initialize the humidity sensor
	
	while(1) {
		if( UCSR0A & (1<<RXC0) ) 
			uartSendByte(1,UDR0);
			
		wdt_reset();
	}
}
/*
int main(void)
{
	UCSR0B &= ~(1<<RXCIE0);	//	don't interrupt for USART0 RX (yet)
	PCMSK3 &= ~(1<<PCINT24);	//	disable pin change interrupt PCINT24 which shared RXD0
	DDRA &= ~0xFF;		//	PORTA (ADC is input for all channels)
	DDRB |= (1<<PB1); PORTB &= ~(1<<PB1);
	open_log_init();	
	open_log_reset_nack();
	mux_init();         //  setup UART1 & set terminal as output
	//gps_init();			//	init the UART0, gps info and NMEA processor
	uart1Init();		//	set up our multiplexed UART1 port
	uartSetBaudRate(1,9600);
	sei();
	
    wdt_disable();			//	disable
	wdt_enable(WDTO_4S);	//	then re-enable the watchdog timer with 4 second interrupt
	
	//	initialize our TIMER0 which counts milliseconds
	DO_AND_WAIT(_init_timer0(),10);
	
	//	some time stamps
	rtc_millis = 0;
	sensor_millis = 0;
	warmer_64Hz_millis = 0;
	
	i2cInit();          //  initialize the I2C bus
	
	dx_indicator_init();
	
	flight_status.serial_channel = MUX_TERMINAL;
	flight_status.terminal_input.state = TERMINAL_WAITING;
	flight_status.terminal_input.timeout = millis() + 5000;		//	five seconds to respond
	flight_status.event.gps_altitude_timeout = millis() + 10000;
	terminal_init();
	read_rtc();
	////////////////////////////////////////////////////////////////////////
	//	I2C bus initialization
	//	Note that for unclear reasons, the BMP085 must be initialized first
	//	followed by the TMP102 sensors.
	/////////////////////////////////////////////////////////////////////////
	_delay_ms(10);				   //  wait until stabilizes
	
	DO_AND_WAIT(_init_bmp085(),5);	//  init the barometric pressure sensor
	DO_AND_WAIT(_init_tmp102(),5);	//	init the temperature monitors
	DO_AND_WAIT(_init_rtc(),5);		//	init the real-time clock
	DO_AND_WAIT(_init_warmers(),2);	//	init the warmers
	hih4030_init();
    while(1)
    {
		uint32_t m = millis();
		
			//	if we are waiting for the terminal input timer to expire and we reach the timeout
			//	then say goodbye to the terminal and redirect the serial output to the OpenLog module
			//
		if( flight_status.terminal_input.state == TERMINAL_WAITING ) {
			if( m >= flight_status.terminal_input.timeout ) {
				//	terminal did not register within timeout, so we will begin regular procedures
				flight_status.terminal_input.state = TERMINAL_OFF;
				uartSendString_P(1,"\rTerminal timed out\r");
				uartSendString_P(1,"Bye\r");
				//  redirect logging to the OpenLog
				#if FORCE_SERIAL_OUTPUT_TERMINAL == 0
				mux_select_channel(MUX_OPEN_LOG);
				#endif
			}	//	terminal wait timed out
			else {
				if( !flight_status.should_ignore_serial_input  ) {
					//	we're waiting for terminal input, so let's get a character
					u08 terminal_data;
					if( uartReceiveByte(1,&terminal_data) ) {
						if( terminal_data != 0x0D )
							uartSendByte(1,(char)terminal_data);
						terminal_process_char( (char)terminal_data );
						flight_status.terminal_input.state = TERMINAL_SELECTED;
					}	//	valid data on terminal
				}	// mux terminal is active channel
			}	//	terminal wait did NOT timeout
		}	//	terminal waiting
		else if( flight_status.terminal_input.state == TERMINAL_OFF ) {
			if( m - rtc_millis >= 1000) {
				read_rtc();
				rtc_millis = m;	
			}	// 1000 ms passed since read RTC	
			if( m - sensor_millis >= 5000) {
				read_sensors();
				report_enviro();
				sensor_millis = m;
			}	//	5000 ms passed since read sensors
			if( m >= flight_status.event.gps_altitude_timeout ) {
				//	TODO: record the GPS altitude in the 'altimeter'
			}
			wdt_reset();
			
			#if USING_WARMERS == 1
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
			#endif
			wdt_reset();
			dx_indicator_update(m);
		} // terminal is not waiting
		else {
				//	NOTE: this code block is everything else that should happen in the main loop
				//	but is not contingent on anything else; so, tasks that should always run
				//	as frequently as possible, e.g. polling the GPS, looking for cell calls, etc.
				//
			if( !flight_status.should_ignore_serial_input ) {
				u08 terminal_data;
				if( uartReceiveByte(1,&terminal_data) ) {
					terminal_process_char( terminal_data );
					if( terminal_data != 0x0D )
						uartSendByte(1,terminal_data);
				} //	valid data on terminal	
			}	//	should not ignore serial data
		}	//	terminal mode
		wdt_reset();
	} //	main loop
}	// main
*/

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
	//uartSendString(1,buffer);
	
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
	char buffer[60];
	sprintf(buffer,"$ENV%02d%02d%02d",rtc.hour,rtc.minute,rtc.second);
    uartSendString(1,buffer);
		
	sprintf(buffer,"IT%02d",internal_temperature.temperature);
    uartSendString(1,buffer);
		
	sprintf(buffer,"ET%02d",external_temperature.temperature);
    uartSendString(1,buffer);
		
	sprintf(buffer,"BP%ldBPT%ld",pressure,temperature);
    uartSendString(1,buffer);
		
	sprintf(buffer,"HUM%03d",humidity); uartSendString(1,buffer);
	uartSendByte(1,"\r");
}

void set_serial_channel(mux_channel_t chan) {
	mux_select_channel(chan);
	flight_status.serial_channel = chan;
}

void set_ignore_serial_data(BOOL state) {
	flight_status.should_ignore_serial_input = state;
}

/*	USER INTERACTION */

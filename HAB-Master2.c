 /////////////////////////////////////////////////////////////////////////////////////////
 //
 //	File		: HAB_Master2.c
 //	Author		: Alan K. Duncan <duncan.alan@mac.com>
 //	Created		: 13 April 2012 10:52:52 PM
 // Modified	: 10 May 2012 06:00:00 AM
 // Version		: 1.0
 //	Target MCU	: ATmega 644
 //
 //	This file is the main program file for the high altitude balloon controller.
 //
 //////////////////////////////////////////////////////////////////////////////////////////

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
#include "peripherals/voltage.h"
#include "gps/gps.h"
#include "capabilities/nmea.h"
#include "sensors/bmp085.h"
#include "sensors/tmp102.h"
#include "sensors/hih4030.h"
#include "common/types.h"
#include "common/pindefs.h"

/************************************************************************/
/* GLOBAL VARIABLES                                                     */
/************************************************************************/
flight_status_t flight_status;			//	our main flight status structure
time_t rtc;								// 	real-time clock
warmer_t battery_warmer;				//	battery warmer structure
tmp102_t internal_temperature;			//	internal temperature sensor
tmp102_t external_temperature;			// 	external temperature sensor
static long temperature = 0;			//	temperature from BP sensor
static long pressure = 0;				//	barometric pressure in Pascals (Pa)
static u08 humidity = 0;				//	external humidity reading
static uint32_t rtc_set_millis = 0;		//	ms to set the rtc by the GPS
static uint32_t rtc_millis = 0;			//	ms to rtc read timeout
static uint32_t sensor_millis = 0;		//	ms to sensor read timeout
static uint32_t warmer_64Hz_millis = 0;	//	ms until next 64 Hz timeout for warmer pulse
static uint32_t position_millis = 0;	//	ms until next position log
static uint8_t warmer_8Hz_div = 0;		//	8 Hz counter for warmer power update

#define POSITION_REPORT_INTERVAL 5000UL	//	interval for recording position
#define RTC_SET_INTERVAL 30000UL		//	interval for resetting RTC by GPS
#define RTC_READ_INTERVAL 1000			//	interval for reading RTC
#define SENSOR_REPORT_INTERVAL 5000UL	//	interval for reporting sensor readings

#define clockCyclesPerMicrosecond() ( F_CPU / 7372800UL )
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
static void _init_timer0(void);

s16 get_internal_temperature();
s16 get_external_temperature();
long barometric_pressure();
u08 get_humidity();
void read_rtc(void);
void report_enviro(void);

/*	FUNCTIONS USED FOR EXTERNAL ACCESS */
void rtc_read_time(time_t *time);
void rtc_set_time(time_t *time);
void set_serial_channel(mux_channel_t chan);
void set_ignore_serial_data(BOOL state);

unsigned long millis();

////////////////////////////////////////////////////////////////////////
//	STATIC INLINE FUNCTIONS
//
//	These functions are broken out of the main run loop for clarity
//
////////////////////////////////////////////////////////////////////////

static void _init_timer1(void) {
	TIMSK1 |= (1<<TOIE1);				//	enable TIMER0 OVF interrupt
	sei();								//	enable global interrupts
	
	TCCR1B |= (1<<CS10);				//	prescaler @ /1	
	TCNT1 = 58162;
}

ISR(TIMER1_OVF_vect)
{
	TCNT1 = 58162;
	timer0_millis++;
	/*
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
	*/
}

static inline void _init_rtc(void) {
	ds1307_init(kDS1307Mode24HR);
	ds1307_sqw_set_mode(k_ds1307_sqw_mode_a);
}

static inline void _init_warmers(void) {
    warmer_controller_init();   // init a2d and pid params for warmers
    warmer_setup();             // initiate warmer(s), defined in warmer_output.h
    warmer_timing_setup();      // setup 64Hz interrupt on TIMER1, def in warmer_timing.h
}

static inline void _init_bmp085(void) {
    bmp085_init();           	//  initialize
}

static inline void _init_tmp102(void) {
    //  setup our internal and external temp sensors
    internal_temperature.address = TMP102_ADDR_GND;
	internal_temperature.location = TMP102_LOC_INT;
	external_temperature.address = TMP102_ADDR_VCC;
	external_temperature.location = TMP102_LOC_EXT;
}

static inline void initialize_i2c_peripherals(void) {
	i2cInit();          //  initialize the I2C bus
	
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
}

static inline void initialize_uart1(void) {
	//	deal with UART1 initialization
 	mux_init();				//	init the serial multiplexer on UART1
	 mux_select_channel(MUX_TERMINAL);
	uart1Init();			//	set up our multiplexed UART1 port
	uartSetBaudRate(1,9600);
	sei();
	uartSendByte(1,0x0C);	//	clear the terminal
	terminal_init();
}

static inline void poll_gps(void) {
	if( UCSR0A & (1<<RXC0) )  { gps_add_char(UDR0); }
	//if( UCSR0A & (1<<RXC0) )  { uartSendByte(1,UDR0); }
	//if( UCSR0A & (1<<RXC0) ) { u08 data = UDR0; if( data == 0x0D ) { uartSendByte(1,'*'); } }
}

static inline void read_sensors(void) {
	tmp102_read_temp(&external_temperature);    // read ext temperature
	tmp102_read_temp(&internal_temperature);
	
	bmp085Convert(&temperature, &pressure);
	
	//	every 15 seconds, measure and compute the temperature-compensated relative humidity 
	if( rtc.second % 15 == 0) {
		humidity = hih4030_compensated_rh(external_temperature.temperature);
	}
}

//	
//	update our warmers if we are using them
//	at 64 Hz we modulate the output to the warmers by using pulse density modulation
//	and at about 8 Hz we check the temperature response (feedback) and change the 
//	power output based on that feedback.  In other words, this function updates the
//	PID controller
//
static inline void update_warmers(uint32_t m) {
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
}

////////////////////////////////////////////////////////////////////////
//
//	REPORTING
//	
//	save various reports to the SD card
//
////////////////////////////////////////////////////////////////////////

//
//	report position/velocity/altitude to UART1 (terminal or logger)
//
static inline void report_position(void) {
	char buffer[60];
	sprintf(buffer,"$POS%02d%02d%02d,%0.5f,%0.5f,%0.1f,%0.1f,%0.1f\r",
		gpsInfo.fix.time.hour,
		gpsInfo.fix.time.minute,
		gpsInfo.fix.time.second,
		gpsInfo.fix.latitude,
		gpsInfo.fix.longitude,
		gpsInfo.fix.altitude,
		gpsInfo.h_track.course,
		gpsInfo.h_track.velocity);
	uartSendString(1,buffer);
}

//
//  send environmental data to whichever UART1 vector is active
//  
static inline void report_enviro(void) {
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
	uartSendByte(1,'\r');
}

#define USING_WARMERS 1
#define FORCE_SERIAL_OUTPUT_TERMINAL 1

int main(void) {
	wdt_disable();			//	disable
	wdt_enable(WDTO_4S);	//	then re-enable the watchdog timer with 4 second interrupt
	
	DDRB |= (1<<PB1); PORTB &= ~(1<<PB1);
	for(u08 i = 0; i < 4; i++) {
		PORTB ^= (1<<PB1);	
		_delay_ms(100);
	}		
	
	//	init the openlog
	open_log_init();	
	open_log_reset_nack();
	
	gps_init();			//	init the UART0, gps info and NMEA processor
	initialize_uart1();	//	init the interface and clear terminal

	_init_timer1();
	_delay_ms(10);
	uint32_t m = millis();
	sensor_millis = m + 2500;		//	stagger our position and sensor reports by 2.5 s
	
	hih4030_init();					//	initialize the humidity sensor
	dx_indicator_init();			//	init dx indicators
	
	initialize_i2c_peripherals();
	
	flight_status.serial_channel = MUX_TERMINAL;
	flight_status.terminal.state = TERMINAL_WAITING;
	flight_status.terminal.timeout = millis() + 10000;		//	five seconds to respond
	flight_status.event.gps_altitude_timeout = millis() + 10000;
	
	while(1) {
		uint32_t m = millis();		//	get our current ms time
		//	if we are waiting for the terminal input timer to expire and we reach the timeout
		//	then say goodbye to the terminal and redirect the serial output to the OpenLog module
		//
		if( flight_status.terminal.state == TERMINAL_WAITING ) {
			if( m >= flight_status.terminal.timeout ) {
				//	terminal did not register within timeout, so we will begin regular procedures
				flight_status.terminal.state = TERMINAL_OFF;
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
						flight_status.terminal.state = TERMINAL_SELECTED;
					}	//	valid data on terminal
				}	// mux terminal is active channel
			}	//	terminal wait did NOT timeout
		}	//	terminal waiting
		else if( flight_status.terminal.state == TERMINAL_OFF ) {
			if( m >= rtc_millis ) {
				read_rtc();
				rtc_millis = m + RTC_READ_INTERVAL;		//	schedule next RTC read
			}	// 1000 ms passed since read RTC	
			if( m >= sensor_millis ) {
				read_sensors();
				report_enviro();
				sensor_millis = m + SENSOR_REPORT_INTERVAL;
			}	//	5000 ms passed since read sensors
			if( m >= flight_status.event.gps_altitude_timeout ) {
				//	TODO: record the GPS altitude in the 'altimeter'
			}
			//	periodically report our position
			if( m >= position_millis ) {
				position_millis = m + POSITION_REPORT_INTERVAL;	//	schedule next interval
				report_position();
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
		//	complete the tasks that are not contingent on anything else
		//	such as polling the GPS, checking for cellular calls, etc
		wdt_reset();				//  kick the watchdog
		static u08 gpsData;
		if( uartReceiveByte(0,&gpsData) ) { gps_add_char(gpsData); }
		
		dx_indicator_update(m);		//	update the dx indicators
		update_warmers(m);			//	update our warmers (e.g. battery etc.)
		
		//	reset RTC by the GPS time at certain interval (nominally 5 minutes)
		if( m > rtc_set_millis ) {
			rtc_set_millis = m + RTC_SET_INTERVAL;
			if( gpsInfo.fix.time.hour == rtc.hour )
				memcpy(&rtc,&gpsInfo.fix.time,sizeof(time_t));
			//	log the reset event
			uartSendString_P(1,"$GPSR");
			char b[10];
			sprintf(b,"%02d%02d%02d\r",rtc.hour,rtc.minute,rtc.second);
			uartSendString(1,b);
		}	//	periodic reset RTC by GPS time
	}
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




/************************************************************************/
/* TIMEKEEPING                                                          */
/************************************************************************/

//
//	read the DS1307
//
void read_rtc(void) {
    rtc.hour = ds1307_hours();
    rtc.minute = ds1307_minutes();
    rtc.second = ds1307_seconds();
}

//	
//	read the RTC and copy the result to time_t pointed to by
//	*time
//
void rtc_read_time(time_t *time) {
	read_rtc();
	memcpy(time,&rtc,sizeof(time_t));
}

//	
//	set the RTC
//
void rtc_set_time(time_t *time) {
	ds1307_set_hours(time->hour);
	ds1307_set_minutes(time->minute);
	ds1307_set_seconds(time->second);
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
	sei();
   
	return m;
}

void set_serial_channel(mux_channel_t chan) {
	mux_select_channel(chan);
	flight_status.serial_channel = chan;
}

void set_ignore_serial_data(BOOL state) {
	flight_status.should_ignore_serial_input = state;
}

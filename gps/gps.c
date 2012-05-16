 /////////////////////////////////////////////////////////////////////////////////////////
 //	File		: gps.c
 //	Author		: Alan K Duncan <duncan.alan@mac.com>
 // Created		: 5/8/2012 9:30:34 AM
 //	Modified	: 5/13/2012 4:25:00 PM
 //	Version		: 1.0
 //	Target MCU	: ATmega 644A
 //
 //	Provides GPS functionality for the HAB project.  Primarily handles the initialization
 //	of the serial interface to the GPS on UART0, accumulating incoming serial characters
 //	from the device; and it provides structures for the parsed GPS data.
 /////////////////////////////////////////////////////////////////////////////////////////
 
#include "../capabilities/uart2.h"
#include "gps.h"
#include "../capabilities/nmea.h"
#include "../capabilities/uart2.h"
#include <stdlib.h>

#define BAUD 4800
#include <util/setbaud.h>

//
//	Initializes our cBuffer, the NMEA processor, and the UART0 
//	peripheral to which the GPS is attached.
//
void gps_init() {
	nmeaInit();				//	initialize our NMEA processor	
	uart0Init();
	uartSetBaudRate(0,4800);
	/*
	UBRR0H = UBRRH_VALUE;		//	set the baud rate 4800 standard for GPS
	UBRR0L = UBRRL_VALUE;
	UCSR0B |= (1<<RXEN0);		//	just want to receive	
	*/
				
}

//
//	Adds a character to the buffer and uses NMEA processor to process 
//	the buffer as candidate packet
//
void gps_add_char(u08 data) {
	nmeaAddChar(data);
}
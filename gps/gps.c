/*
 * gps.c
 *
 * Created: 5/8/2012 9:30:47 AM
 *  Author: Administrator
 */ 


#include "../capabilities/uart2.h"
#include "gps.h"
#include "../capabilities/nmea.h"
#include <stdlib.h>

#define BAUD 4800
#include <util/setbaud.h>

//
//	Initializes our cBuffer, the NMEA processor, and the UART0 
//	peripheral to which the GPS is attached.
//
void gps_init() {
	bufferInit(&gpsBuffer,c_gpsData,BUFFER_SIZE);	//	initialize the cBuffer that holds our GPS data incoming
	nmea_init();									//	initialize our NMEA processor	
	UBRR0H = UBRRH_VALUE;		//	set the baud rate 4800 standard for GPS
	UBRR0L = UBRRL_VALUE;
	UCSR0B |= (1<<RXEN0);		//	just want to receive				
}

//
//	Adds a character to the buffer and uses NMEA processor to process 
//	the buffer as candidate packet
//
void gps_add_char(u08 data) {
	nmeaAddChar(data);
}
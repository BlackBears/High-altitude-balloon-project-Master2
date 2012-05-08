/*
 * gps.c
 *
 * Created: 5/8/2012 9:30:47 AM
 *  Author: Administrator
 */ 


#include "../capabilities/uart-644a.h"
#include "gps.h"
#include "../capabilities/nmea.h"
#include <stdlib.h>

#define BUFFER_SIZE 100

unsigned char *c_gpsData;
//
//	Initializes our cBuffer, the NMEA processor, and the UART0 
//	peripheral to which the GPS is attached.
//
void gps_init() {
	c_gpsData = malloc(100 * sizeof(unsigned char));
	bufferInit(gpsBuffer,c_gpsData,BUFFER_SIZE);	//	initialize the cBuffer that holds our GPS data incoming
	nmea_init();								//	initialize our NMEA processor
	uart_init(4800);							//	initialize the UART0 with 4800 baud
}

//
//	Adds a character to the buffer and uses NMEA processor to process 
//	the buffer as candidate packet
//
void gps_add_char(u08 data) {
	//bufferAddToEnd(gpsBuffer,(unsigned char)data);
	//nmea_process(gpsBuffer);
}
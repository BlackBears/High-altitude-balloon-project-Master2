//////////////////////////////////////////////////////////////////////////////////////////
//
//	File		: nmea.c
//	Author		: Alan K. Duncan <duncan.alan@mac.com>
//	Created		: 5/8/2012 9:37:34 AM
//	Modified	: 5/13/2012 4:32:00 PM
//	Version		: 1.0
//	Target MCU	: ATmega 644A
//
//	This file provides NMEA parsing functionality.  At this time, it parses only two NMEA
//	sentences - $GPGGA and $GPVTG which are the only two pieces of functionality required
//	for the HAB project.  This set of functions operates as a finite state machine parser.
//
//////////////////////////////////////////////////////////////////////////////////////////

#include "nmea.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "../common/global.h"
#include "buffer.h"
#include "rprintf.h"
#include "../gps/gps.h"
#include "uart2.h"

#define BUFFER_LEN 80
#define NMEA_DEBUG 1

//	shorthand macro to skip to the next comma-delimited field
#define SKIP_TO_NEXT_FIELD_IN_PACKET while(packet[i++] != ',');
#define RESET_ON_BUFFER_OVERFLOW if( idx > BUFFER_LEN ) { idx = 0; state = NMEA_IDLE; }

//	set DEBUG_NMEA to 1 to log parsing events, state changes, etc.
#define DEBUG_NMEA 1
#if DEBUG_NMEA == 1
#define DEBUG_PRINT(a) printf(a)
#else
#define DEBUG_PRINT(a) //
#endif

//	
//	state machine for the NMEA parser
//
nmeaState state;

uint8_t process_vtg_packet();
uint8_t process_gga_packet();

char packet[BUFFER_LEN];
uint8_t idx;

void nmeaInit(void) {
    state = NMEA_IDLE;		//	state machine starts in idle
}

void nmeaAddChar(uint8_t data) {
    switch( state ) {
        case NMEA_IDLE:
            if( data == '$' ) {
            	//	$ is start character, start condition set
                state = NMEA_START;
                DEBUG_PRINT("Detected NMEA_START\r");
                idx = 0;
                packet[idx++] = data;
                
                RESET_ON_BUFFER_OVERFLOW
            }
            break;
        case NMEA_START:
        	//	continue accumulating
            state = NMEA_ACCUMULATE;
            DEBUG_PRINT("NMEA_ACCUMULATE\r");
            packet[idx++] = data;
            RESET_ON_BUFFER_OVERFLOW
            break;
        case NMEA_ACCUMULATE:
            if( data == '\r' ) {
                state = NMEA_COMPLETE;
#if DEBUG_NMEA
                printf("NMEA_COMPLETE: %d\r",idx);
#endif
                if( strstr(packet, "VTG") ) {
                    DEBUG_PRINT("VTG PACKET FOUND\r");
                    process_vtg_packet();
                }
                else if( strstr(packet, "GGA") ) {
                    DEBUG_PRINT("GGA PACKET FOUND\r");
                    process_gga_packet();
                }
                state = NMEA_IDLE;
                idx = 0;
                DEBUG_PRINT("NMEA_IDLE\r");
            }
            else {
                packet[idx++] = data;
                RESET_ON_BUFFER_OVERFLOW
                break;
            }
            break;
    }
}

uint8_t process_vtg_packet() {
    uint8_t i = 7;	//begin immediately after the "$GPRMC"
    //	attempt to reject empty packets right away
    if( packet[i] == ',' && packet[i+1] == ',') { return 0;}
    
    gpsInfo.h_track.course = atof(&packet[i]);
    printf("\tDEGREES: %0.1f\r",gpsInfo.h_track.course);
    
    
    //	next field is T for track
    SKIP_TO_NEXT_FIELD_IN_PACKET;
    //	then mag track
    SKIP_TO_NEXT_FIELD_IN_PACKET;
    // then M for magnetic
    SKIP_TO_NEXT_FIELD_IN_PACKET
    // then GS in knots
    SKIP_TO_NEXT_FIELD_IN_PACKET
    gpsInfo.h_track.velocity = atof(&packet[i]);
    
    printf("\tVELOCITY = %0.1f\r",gpsInfo.h_track.velocity);
    
    return 1;
}

uint8_t process_gga_packet() {
    char temp[6];
    uint8_t i_temp,i_temp2;
    uint8_t i = 7;	//begin immediately after the "$GPGGA"
    //	attempt to reject empty packets right away
    if( packet[i] == ',' && packet[i+1] == ',') { return 0;}
    
    //	first field is hour UTC
    strncpy(temp,&packet[i],2);
    gpsInfo.fix.time.hour = atoi(temp);
    i += 2;	//	advance to minutes time UTC
    strncpy(temp,&packet[i],2);
    gpsInfo.fix.time.minute = atoi(temp);
    i += 2; //	advance to seconds time UTC
    strncpy(temp,&packet[i],2);
    gpsInfo.fix.time.second = atoi(temp);
    
    
    SKIP_TO_NEXT_FIELD_IN_PACKET;	// next field is latitude
    strncpy(temp,&packet[i],2);
    gpsInfo.fix.latitude = atof(temp);
    i += 2;		// advance to latitude minutes;
    strncpy(temp,&packet[i],2);
    i_temp = atoi(temp);	// i_temp is the minutes latitude
    SKIP_TO_NEXT_FIELD_IN_PACKET;	// advance to fractional minutes
    i_temp2 = atoi(&packet[i]);		// fractional minutes
    gpsInfo.fix.latitude += (float)i_temp/60.0f + (float)i_temp2/3600.0f;
    
    SKIP_TO_NEXT_FIELD_IN_PACKET;	// next field is N/s indicator
    if( packet[i] == 'S' ) { gpsInfo.fix.latitude = -gpsInfo.fix.latitude; }
    
    SKIP_TO_NEXT_FIELD_IN_PACKET;	// next field if longitude
    strncpy(temp,&packet[i],3); temp[3] = '\0';
    gpsInfo.fix.longitude = atof(temp);
    i += 3;		// advance to longitude minutes;
    strncpy(temp,&packet[i],2); temp[2] = '\0';
    i_temp = atoi(temp);	// i_temp is the minutes longitude
    SKIP_TO_NEXT_FIELD_IN_PACKET;	// advance to fractional minutes
    i_temp2 = atoi(&packet[i]);		// fractional minutes
    gpsInfo.fix.longitude += (float)i_temp/60.0f + (float)i_temp2/3600.0f;
    
    // next field is the E/W indicator
   SKIP_TO_NEXT_FIELD_IN_PACKET;
    if( packet[i] == 'W' ) { gpsInfo.fix.longitude = -gpsInfo.fix.longitude; }
    
    printf("degrees is %0.6f\r",gpsInfo.fix.longitude);
    
    // next field is the fix quality
    SKIP_TO_NEXT_FIELD_IN_PACKET;
    printf("SAT QUAL = %d\r",atoi(&packet[i]));
    
    // next field is the satellite number
    SKIP_TO_NEXT_FIELD_IN_PACKET;
    printf("SAT NUM = %d\r",atoi(&packet[i]));
    
    // next field is the h dilution of position
    SKIP_TO_NEXT_FIELD_IN_PACKET;
    printf("H DIL POS = %0.1f\r",atof(&packet[i]));
    
    // next field is the altitude in meters
    SKIP_TO_NEXT_FIELD_IN_PACKET;
    gpsInfo.fix.altitude = atof(&packet[i]);
    
    printf("ALT = %0.1fm\r",gpsInfo.fix.altitude);
    return 1;

}

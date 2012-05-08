/*
 * nmea.c
 *
 * Created: 5/8/2012 9:37:44 AM
 *  Author: Administrator
 */ 


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

#define SKIP_TO_NEXT_FIELD_IN_PACKET while(packet[i++] != ',')
// Program ROM constants

// Global variables
extern gps_info_t gps_info;
u08 nmea_packet[NMEA_BUFFERSIZE];

void nmea_process_GPGGA(u08 *packet);
void nmea_process_GPVTG(u08 *packet);

void nmea_init(void)
{
	// currently empty implementation
}

u08* nmea_get_packet_buffer(void)
{
	return nmea_packet;
}

u08 nmea_process(cBuffer* rxBuffer)
{
	u08 foundpacket = NMEA_NODATA;
	u08 startFlag = FALSE;
	//u08 data;
	u16 i,j;

	// process the receive buffer
	// go through buffer looking for packets
	while(rxBuffer->datalength) {
		// look for a start of NMEA packet
		if(bufferGetAtIndex(rxBuffer,0) == '$') {
			// found start
			startFlag = TRUE;
			// when start is found, we leave it intact in the receive buffer
			// in case the full NMEA string is not completely received.  The
			// start will be detected in the next nmeaProcess iteration.

			// done looking for start
			break;
		}
		else
			bufferGetFromFront(rxBuffer);
	}
	
	// if we detected a start, look for end of packet
	if(startFlag) {
		for(i=1; i<(rxBuffer->datalength)-1; i++) {
			// check for end of NMEA packet <CR><LF>
			if((bufferGetAtIndex(rxBuffer,i) == '\r') && (bufferGetAtIndex(rxBuffer,i+1) == '\n')) {
				// have a packet end
				// dump initial '$'
				bufferGetFromFront(rxBuffer);
				// copy packet to nmea_packet
				for(j=0; j<(i-1); j++) {
					// although NMEA strings should be 80 characters or less,
					// receive buffer errors can generate erroneous packets.
					// Protect against packet buffer overflow
					if(j<(NMEA_BUFFERSIZE-1))
						nmea_packet[j] = bufferGetFromFront(rxBuffer);
					else
						bufferGetFromFront(rxBuffer);
				}
				// null terminate it
				nmea_packet[j] = 0;
				// dump <CR><LF> from rxBuffer
				bufferGetFromFront(rxBuffer);
				bufferGetFromFront(rxBuffer);

				#ifdef NMEA_DEBUG_PKT
				rprintf("Rx NMEA packet type: ");
				rprintfStrLen(nmea_packet, 0, 5);
				rprintfStrLen(nmea_packet, 5, (i-1)-5);
				rprintfCRLF();
				#endif
				// found a packet
				// done with this processing session
				foundpacket = NMEA_UNKNOWN;
				break;
			}
		}
	}

	if( foundpacket )
	{
		// check message type and process appropriately
		if(!strncmp(nmea_packet, "GPGGA", 5)) {
			// process packet of this type
			nmea_process_GPGGA(nmea_packet);
			// report packet type
			foundpacket = NMEA_GPGGA;
		}	// process found $GPGGA packet
		else if(!strncmp(nmea_packet, "GPVTG", 5)) {
			// process packet of this type
			nmea_process_GPVTG(nmea_packet);
			// report packet type
			foundpacket = NMEA_GPVTG;
		}	//	process found $GPVTG packet
	}
	else if(rxBuffer->datalength >= rxBuffer->size) {
		// if we found no packet, and the buffer is full
		// we're logjammed, flush entire buffer
		bufferFlush(rxBuffer);
	}
	return foundpacket;
}

void nmea_process_GPGGA(u08 *packet) {
	char *endptr;	//	pointer to the end of processed bit
	char temp[6];
	u08 i = 6;	//	begin parsing just after the GPGGA
	u08 i_temp,i_temp2;	//	scratchpad
	//	attempt to reject empty packets right away
	if( packet[i] == "," && packet[i+i] == "," ) { return; }
	//	first field is hour UTC
	strncpy(temp,&packet[i],2);
	gps_info.fix.time.hour = atoi(temp);
	//	next is minutes UTC
	i += 2;
	strncpy(temp,&packet[i],2);
	gps_info.fix.time.minute = atoi(temp);
	//	next field is seconds UTC
	i += 2;
	strncpy(temp,&packet[i],2);
	gps_info.fix.time.second = atoi(temp);
	
	//	next field is latitude
	SKIP_TO_NEXT_FIELD_IN_PACKET;
	strncpy(temp,&packet[i],2);
	gps_info.fix.latitude = atof(temp);	//	whole degrees latitude
	i += 2;	//	advance to latitude minutes;
	strncpy(temp,&packet[i],2);
	//i_temp = atoi(temp);	//	i_temp is the minutes latitude
	while( packet[i++] != '.' );	//	advance to fractional minutes
	//i_temp2 = atoi(&packet[i]);		//	fractional minutes
	gps_info.fix.latitude += (float)i_temp/60.0f + (float)i_temp2/3600.0f;
	
	//	next field is the N/S indicator for latitude
	SKIP_TO_NEXT_FIELD_IN_PACKET;
	if( packet[i] = 'S') { gps_info.fix.latitude = -gps_info.fix.latitude; }
	
	//	next field is the longitude
	SKIP_TO_NEXT_FIELD_IN_PACKET;
	strncpy(temp,&packet[i],3); temp[3] = '\0';
	//gps_info.fix.longitude = atof(temp);	//	whole degrees longitude
	i += 3;	//	advance to latitude minutes;
	strncpy(temp,&packet[i],2); temp[2] = '\0';
	//i_temp = atoi(temp); 	//	i_temp is the minutes latitude
	while( packet[i++] != '.' );	//	advance to fractional minutes
	//i_temp2 = atoi(&packet[i]);		//	fractional minutes
	gps_info.fix.longitude += (float)i_temp/60.0f + (float)i_temp2/3600.0f;
	
	//	next field is the E/W indicator for longitude
	SKIP_TO_NEXT_FIELD_IN_PACKET;
	if( packet[i] = 'W') { gps_info.fix.longitude = -gps_info.fix.longitude; }
		
	//	next field is the fix quality
	SKIP_TO_NEXT_FIELD_IN_PACKET;
	
	//	next field is the satellite count
	SKIP_TO_NEXT_FIELD_IN_PACKET;
	
	//	next field is the h dilution of position
	SKIP_TO_NEXT_FIELD_IN_PACKET;
	
	//	next field is the altitude in meters
	SKIP_TO_NEXT_FIELD_IN_PACKET;
	//gps_info.fix.altitude = atof(&packet[i]);

}

void nmea_process_GPVTG(u08 *packet) {
	char *endptr;	//	pointer to the end of processed bit
	char temp[6];
	u08 i = 6;	//	begin parsing just after the sentence identifier
	
	// attempt to reject empty packets right away
	if( packet[i] == "," && packet[i+i] == "," ) { return; }
	
	//	first field is the true track made good in degrees
	gps_info.h_track.track_angle = atof(&packet[i]);
	
	//	next field is T for track
	SKIP_TO_NEXT_FIELD_IN_PACKET;
	//	next is the magnetic track
	SKIP_TO_NEXT_FIELD_IN_PACKET;
	//  next field is the M for magnetic
	SKIP_TO_NEXT_FIELD_IN_PACKET;
	//	next field is the GS in knots
	SKIP_TO_NEXT_FIELD_IN_PACKET; 
	gps_info.h_track.velocity = atof(&packet[i]);
	
	//	we can skip the rest of the packet
}

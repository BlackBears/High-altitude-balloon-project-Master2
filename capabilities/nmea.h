/*
 * nmea.h
 *
 * Created: 5/8/2012 9:37:34 AM
 *  Author: Administrator
 */ 


#ifndef NMEA_H_
#define NMEA_H_

#include "../common/global.h"
#include "../common/types.h"
#include "buffer.h"

// constants/macros/typdefs
#define NMEA_BUFFERSIZE		80

// Message Codes
#define NMEA_NODATA		0	// No data. Packet not available, bad, or not decoded
#define NMEA_GPGGA		1	// Global Positioning System Fix Data
#define NMEA_GPVTG		2	// Course over ground and ground speed
#define NMEA_GPGLL		3	// Geographic position - latitude/longitude
#define NMEA_GPGSV		4	// GPS satellites in view
#define NMEA_GPGSA		5	// GPS DOP and active satellites
#define NMEA_GPRMC		6	// Recommended minimum specific GPS data
#define NMEA_UNKNOWN	0xFF// Packet received but not known

// Debugging
//#define NMEA_DEBUG_PKT	///< define to enable debug of all NMEA messages
//#define NMEA_DEBUG_GGA	///< define to enable debug of GGA messages
//#define NMEA_DEBUG_VTG	///< define to enable debug of VTG messages	

// functions
void nmea_init(void);
u08* nmea_get_packet_buffer(void);
u08 nmea_process(cBuffer* rxBuffer);
void nmea_process_GPGGA(u08* packet);
void nmea_process_GPVTG(u08* packet);


#endif /* NMEA_H_ */
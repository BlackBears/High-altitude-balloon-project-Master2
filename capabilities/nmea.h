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

enum {
    NMEA_IDLE,
    NMEA_START,
    NMEA_PROVISIONAL,
    NMEA_ACCUMULATE,
    NMEA_COMPLETE
};
typedef uint8_t nmeaState;

//
//	initialize the UART0 interface for the GPS
//
void nmeaInit(void);

//	
//	add an incoming character to be analyzed
/
void nmeaAddChar(uint8_t data);

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
#define NMEA_UNKNOWN	0xFF  // Packet received but not known




#endif /* NMEA_H_ */
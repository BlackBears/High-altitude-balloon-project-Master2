 /////////////////////////////////////////////////////////////////////////////////////////
 //	File		: gps.h
 //	Author		: Alan K Duncan <duncan.alan@mac.com>
 // Created		: 5/8/2012 9:30:34 AM
 //	Modified	: 5/13/2012 4:25:00 PM
 // Version		: 1.0
 //	Target MCU	: ATmega 644A
 //
 //	Provides GPS functionality for the HAB project.  Primarily handles the initialization
 //	of the serial interface to the GPS on UART0, accumulating incoming serial characters
 //	from the device; and it provides structures for the parsed GPS data.
 /////////////////////////////////////////////////////////////////////////////////////////
 


#ifndef GPS_H_
#define GPS_H_

#include "../common/global.h"
#include "../common/types.h"
#include "../capabilities/buffer.h"

typedef struct {
	float latitude;
	float longitude;
	float altitude;
	time_t time;
	uint8_t valid;
} gps_fix_t;

typedef struct {
	float velocity;			//	ground speed in knots
	float course;			//	course in degrees
} gps_course_t;

typedef struct {
	gps_fix_t fix;			//	fix
	gps_course_t h_track;	//	horizontal course (from RMC sentence)
} gps_info_t;

gps_info_t gpsInfo;

void gps_init();
void gps_add_char(u08 data);

#endif /* GPS_H_ */
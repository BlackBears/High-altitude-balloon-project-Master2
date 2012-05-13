/*
 * gps.h
 *
 * Created: 5/8/2012 9:30:34 AM
 *  Author: Administrator
 */ 


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
/*
 * types.h
 *
 * Created: 4/13/2012 11:08:36 PM
 *  Author: Administrator
 */ 


#ifndef TYPES_H_
#define TYPES_H_

#include "global.h"

enum {
	k_state_preflight,
	k_state_ascent,
	k_state_descent
};
typedef u08 flight_state_t;

enum {
	k_system_state_configure,
	k_system_state_self_test,
	k_system_state_run
};
typedef u08 system_state_t;

typedef struct {
	system_state_t system_state;
	flight_state_t flight_state;
} global_states_t;

typedef struct {
    u08 hour;
    u08 minute;
    u08 second;
    BOOL new_second;
} time_t;

typedef struct {
    float latitude;		//	fractional degrees
	float longitude;	//	fractional degrees
	float altitude;		//	meters
	BOOL valid;			//	is this a valid fix?
    time_t time;
} gps_fix_t;

typedef struct {
	float velocity;		//	horizontal speed (ground speed in knots)
	float track_angle;	//	course
} gps_course_t;

typedef struct {
	gps_fix_t fix;			//	fix
	gps_course_t h_track;	//	horizontal track
} gps_info_t;


#endif /* TYPES_H_ */

#ifndef __STATES_H_
#define __STATES_H_

#include "global.h"
#include "types.h"
#include "../peripherals/mux.h"

enum {
	k_flight_mode_off,	
	k_flight_mode_preflight,
	k_flight_mode_ascent,
	k_flight_mode_descent,
	k_flight_mode_landed
};
typedef u08 flight_mode_t;

enum {
    k_peripheral_status_error,
    k_peripheral_status_ok, 
    k_peripheral_status_indeterminate
};
typedef u08 peripheral_status_t;

#define PWR_ON  1
#define PWR_OFF 0


enum {
	TERMINAL_WAITING,
	TERMINAL_SELECTED,
	TERMINAL_OFF
};
typedef u08 terminal_status_t;

typedef struct {
	terminal_status_t state;
	uint32_t timeout;
} terminal_input_t;

typedef struct {
	u32 gps_altitude_timeout;	//	when should we record the GPS-provided altitude in the altimeter
} periodic_timing_t;

typedef struct {
	flight_mode_t flight_mode;
	mux_channel_t serial_channel;		//  current vector of serial out
	BOOL should_ignore_serial_input;	//	used to suppress interpretation of serial data
	gps_fix_t current_fix;
	terminal_input_t terminal;
	periodic_timing_t event;
} flight_status_t;

#endif
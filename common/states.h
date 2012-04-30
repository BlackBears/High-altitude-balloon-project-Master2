
#ifndef __STATES_H_
#define __STATES_H_

#include "global.h"

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

typedef struct {
    peripheral_status_t status;
    u08 power;                      //  PWR_ON or PWR_OFF
    u08 connect_attempts;           //  # times attempted to connect to slave
    
} sensor_status_t;

enum {
    k_serial_out_open_log,
    k_serial_out_lcd,
    k_serial_out_terminal
};
typedef serial_out_channel_t;

typedef struct {
    u08 hour;
    u08 minute;
    u08 second;
    BOOL new_second;
} time_t;

typedef struct {
    char latitude[10];      //  4807.038N
    char longitude[11];     //  01131.000E
    time_t time;
} gps_fix_t;

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
	flight_mode_t flight_mode;
	serial_out_channel_t serial_channel;    //  current vector of serial out
	gps_fix_t current_fix;
	terminal_input_t terminal_input;
} flight_status_t;

#endif
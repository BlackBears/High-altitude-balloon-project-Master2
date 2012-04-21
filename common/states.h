

#include "global.h"

enum {
	k_flight_mode_off,	
	k_flight_mode_preflight,
	k_flight_mode_ascent,
	k_flight_mode_descent,
	k_flight_mode_landed
};
typedef u08 flight_mode_t;

typedef struct {
	flight_mode_t flight_mode;
} flight_status_t;
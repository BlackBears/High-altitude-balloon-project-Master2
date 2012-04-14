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



#endif /* TYPES_H_ */
//////////////////////////////////////////////////////////////////////////////////////////
//	
//	File		: 'cutdown.h'
//	Author		: Alan K. Duncan <duncan.alan@mac.com>
//	Created		: 2012-05-01
//	Revised		: 2012-05-09
//	Version		: 1.0
//	Target MCU	: ATmega644A
//	
//	This file handles the payload cutdown.  If the payload must be separated from the
//  balloon, the functions in this file handles the separation.  The cutdown command must
//	be confirmed before the actual burn can take place.
//
//
//////////////////////////////////////////////////////////////////////////////////////////

#ifndef __CUTDOWN_H
#define __CUTDOWN_H

#include "../common/global.h"

#define CUTDOWN_CONTROLLER_REQUEST_LATENCY      60000  //  must confirm in 60s
#define CUTDOWN_CONTROLLER_BURN_DURATION        20000  //  burn for 20s

enum {
    CUTDOWN_IDLE,
    CUTDOWN_REQUESTED,
    CUTDOWN_CONFIRMED,
    CUTDOWN_BURN,
    CUTDOWN_COMPLETED
};
typedef u08 cutdown_state_t;

uint32_t cutdown_timeout;			//	timeout for current state in milliseconds
cutdown_state_t cutdown_state;		//	our current state

void cutdown_controller_init(void);
void cutdown_controller_request(uint32_t m);
void cutdown_controller_confirm(uint32_t m);
void cutdown_controller_update(uint32_t m);

#endif
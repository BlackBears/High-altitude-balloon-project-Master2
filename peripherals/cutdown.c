//////////////////////////////////////////////////////////////////////////////////////////
//	
//	File		: 'cutdown.c'
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

#include "cutdown.h"
#include "../common/pindefs.h"

void cutdown_controller_init(void) {
    DDR(CUTDOWN_CONTROL_PORT) |= (1<<CUTDOWN_CONTROL_PIN);
    CUTDOWN_CONTROL_PORT &= ~(1<<CUTDOWN_CONTROL_PIN);
    cutdown_state = CUTDOWN_IDLE;
    cutdown_timeout = 0;
}

void cutdown_controller_update(uint32_t m) {
	if( m > cutdown_timeout ) {
		switch( cutdown_state ) {
			case CUTDOWN_REQUESTED: {
				cutdown_state = CUTDOWN_IDLE;	//	never confirmed
				cutdown_timeout = m;
				break;
			}
			case CUTDOWN_BURN: {
				cutdown_state = CUTDOWN_COMPLETED;
				CUTDOWN_CONTROL_PORT &= ~(1<<CUTDOWN_CONTROL_PIN);
				cutdown_timeout = m;
				break;
			}	
			
		}
	}
}

void cutdown_controller_request(uint32_t m) {
    cutdown_state = CUTDOWN_REQUESTED;
    cutdown_timeout = m + CUTDOWN_CONTROLLER_REQUEST_LATENCY;
}

void cutdown_controller_confirm(uint32_t m) {
    cutdown_state = CUTDOWN_CONFIRMED;
    CUTDOWN_CONTROL_PORT |= (1<<CUTDOWN_CONTROL_PIN);
    cutdown_state = CUTDOWN_BURN;
    cutdown_timeout = m + CUTDOWN_CONTROLLER_BURN_DURATION ;
}


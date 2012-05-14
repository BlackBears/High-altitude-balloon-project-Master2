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
    cutdown.state = k_cutdown_state_idle;
    cutdown.ticks = 0;
}

void cutdown_controller_update(void) {
    switch( cutdown.state ) {
        case k_cutdown_state_requested: {
            if( cutdown.ticks > CUTDOWN_CONTROLLER_REQUEST_LATENCY ) {
                cutdown.ticks = 0;
                cutdown.state = k_cutdown_state_idle;
            }
            else { cutdown.ticks = cutdown.ticks + 1; }
            break;
        }
        case k_cutdown_state_burn: {
            if( cutdown.ticks > CUTDOWN_CONTROLLER_BURN_DURATION ) {
                CUTDOWN_CONTROL_PORT &= ~(1<<CUTDOWN_CONTROL_PIN);
                cutdown.state = k_cutdown_state_burn_completed;
                cutdown.ticks = 0;
            }
            else { cutdown.ticks = cutdown.ticks + 1; }
            break;
        }
        default:
            break;
    }
}

void cutdown_controller_request(void) {
    cutdown.state = k_cutdown_state_requested;
    cutdown.ticks = 0;
}

void cutdown_controller_confirm(void) {
    cutdown.state = k_cutdown_state_confirmed;
    CUTDOWN_CONTROL_PORT |= (1<<CUTDOWN_CONTROL_PIN);
    cutdown.state = k_cutdown_state_burn;
    cutdown.ticks = 0;
}


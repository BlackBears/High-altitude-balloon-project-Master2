
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


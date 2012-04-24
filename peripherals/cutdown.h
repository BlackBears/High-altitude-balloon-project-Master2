

#ifndef __CUTDOWN_H
#define __CUTDOWN_H

#include "../common/global.h"

#define CUTDOWN_CONTROLLER_REQUEST_LATENCY      60  //  must confirm in 60s
#define CUTDOWN_CONTROLLER_BURN_DURATION        20  //  burn for 20s

enum {
    k_cutdown_state_idle,
    k_cutdown_state_requested,
    k_cutdown_state_confirmed,
    k_cutdown_state_burn,
    k_cutdown_state_burn_completed
};
typedef u08 cutdown_state_t;

typedef struct {
    cutdown_state_t state;
    u08 ticks
} cutdown_controller_t;

cutdown_controller_t cutdown;

void cutdown_controller_init(void);
void cutdown_controller_request(void);
void cutdown_controller_confirm(void);
void cutdown_controller_update(void);

#endif
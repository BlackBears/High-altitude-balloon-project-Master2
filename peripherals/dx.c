

#include "dx.h"
#include <avr/io.h>
#include "../common/pindefs.h"

/*  FUNCTION PROTOTYPES */
void dx_indicator_set_state(u08 index, BOOL state);

void dx_indicator_init(void) {
	DDR(DX_1_PORT) |= (1<<DX_1_PIN);
    DDR(DX_2_PORT) |= (1<<DX_2_PIN);
    DDR(DX_3_PORT) |= (1<<DX_3_PIN);

    for(u08 i = 0; i < DX_INDICATOR_COUNT; i++) {
        dx_indicator_set_state(i,FALSE);
        dx_indicators[i].count = 0;
    }
}
void dx_indicator_flash(u08 index,u08 count,u32 m) {
    dx_indicator_set_state(index,TRUE);
    dx_indicators[index].count = count;
    dx_indicators[index].last_m = m;
}

void dx_indicator_update(u32 m) {
    for(u08 i = 0; i < DX_INDICATOR_COUNT; i++) {
        if( dx_indicators[i].count != 0 ) {
            if( m >= dx_indicators[i].last_m + 500 ) {
                if( dx_indicators[i].state ) {
                    dx_indicator_set_state(i,FALSE);
                    dx_indicators[i].count--;
                }
                else
                    dx_indicator_set_state(i,TRUE);
            }   //  transition time
        }
        else
            dx_indicator_set_state(i,FALSE);
    }   // indicators iteration
}

void dx_indicator_set_state(u08 index, BOOL state) {
    switch( index )
    {
        case 0:
            if( state )
                DX_1_PORT |= (1<<DX_1_PIN);
            else
                DX_1_PORT &= ~(1<<DX_1_PIN);
            break;
        case 1:
            if( state )
                DX_2_PORT |= (1<<DX_2_PIN);
            else
                DX_2_PORT &= ~(1<<DX_2_PIN);
            break;
        case 2:
            if( state )
                DX_3_PORT |= (1<<DX_3_PIN);
            else
                DX_3_PORT &= ~(1<<DX_3_PIN);
            break;
    }
    dx_indicators[index].state = state;
}


#include "mux.h"
#include "../common/pindefs.h"

#define MUX_A_HIGH SERIAL_MUX_A_PORT |= (1<<SERIAL_MUX_A_PIN)
#define MUX_B_HIGH SERIAL_MUX_B_PORT |= (1<<SERIAL_MUX_B_PIN)
#define MUX_A_LOW SERIAL_MUX_A_PORT &= ~(1<<SERIAL_MUX_A_PIN)
#define MUX_B_LOW SERIAL_MUX_B_PORT &= ~(1<<SERIAL_MUX_B_PIN)

void mux_init(void) {
    DDR(SERIAL_MUX_A_PORT) |= (1<<SERIAL_MUX_A_PIN);
    DDR(SERIAL_MUX_B_PORT) |= (1<<SERIAL_MUX_B_PIN);
    
    mux_select_channel(mux_terminal);
}

void mux_select_channel(mux_channel_t chan) {
    if( chan = mux_open_log ) {
        MUX_A_LOW;
        MUX_B_LOW;
    }
    switch( chan ) {
        case mux_open_log:
            MUX_A_LOW;
            MUX_B_LOW;
            break;
        case mux_lcd:
            MUX_A_HIGH;
            MUX_B_LOW;
            break;
        case mux_terminal:
            MUX_A_LOW;
            MUX_B_HIGH;
            break;
    }
}
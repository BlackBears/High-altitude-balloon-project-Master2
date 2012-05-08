

#include "mux.h"
#include "../common/pindefs.h"

#define MUX_A_HIGH SERIAL_MUX_A_PORT |= (1<<SERIAL_MUX_A_PIN)
#define MUX_B_HIGH SERIAL_MUX_B_PORT |= (1<<SERIAL_MUX_B_PIN)
#define MUX_A_LOW SERIAL_MUX_A_PORT &= ~(1<<SERIAL_MUX_A_PIN)
#define MUX_B_LOW SERIAL_MUX_B_PORT &= ~(1<<SERIAL_MUX_B_PIN)

void mux_init(void) {
    DDR(SERIAL_MUX_A_PORT) |= (1<<SERIAL_MUX_A_PIN);
    DDR(SERIAL_MUX_B_PORT) |= (1<<SERIAL_MUX_B_PIN);
    
    mux_select_channel(MUX_TERMINAL);
	mux_current_channel = MUX_TERMINAL;
}

void mux_select_channel(mux_channel_t chan) {
	if( chan == MUX_OPEN_LOG ) {
		MUX_A_LOW;
        MUX_B_LOW;	
	}		
	else if( chan == MUX_TERMINAL ) {
		MUX_A_LOW;
        MUX_B_HIGH;
	}		
	mux_current_channel = chan;
}
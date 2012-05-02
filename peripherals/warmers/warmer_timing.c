

#include "warmer_timing.h"
#include "warmer.h"
#include "warmer_output.h"

u08 div;

//
//  setup a 64 Hz timer
//
void warmer_timing_setup(void) {
	DDRB |= (1<<PB0);
    div = 0;                    //  64 Hz cycles are divided into 8 equal sectionss
	sei();
}

//
//  TIMER1 overflow interrupt vector
//
ISR(INT6_vect) {
    //warmer_update_64Hz();       //  update the conroller output at 64Hz
    PORTB ^= (1<<PB0);
    //  execute control update every 8 steps (64 Hz/8 = 8 Hz)
    if( ++div == 8 ) {
        div = 0;
        //warmer_update_8Hz();    //  measure temperature and adjust via PID at 8 Hz
    }
}
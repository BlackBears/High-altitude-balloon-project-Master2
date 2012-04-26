

#include "warmer_timing.h"
#include "warmer.h"
#include "warmer_output.h"

u08 div;

//
//  setup a 64 Hz timer
//
void warmer_timing_setup(void) {
    div = 0;                    //  64 Hz cycles are divided into 8 equal sections
    
    TCCR1A = 0x00;
    TIMSK |= (1<<TOIE1);
    sei();
    TCNT1 = 34285; 
    TCCR1B |= (1<<CS11);        //  prescaler at Fcpu/8
    
}

//
//  TIMER1 overflow interrupt vector
//
ISR(TIMER1_OVF_vect) {
    warmer_update_64Hz();       //  update the conroller output at 64Hz
    
    //  re-enable interrupts
    sei();
    
    //  execute control update every 8 steps (64 Hz/8 = 8 Hz)
    if( ++div == 8 ) {
        div = 0;
        warmer_update_8Hz();    //  measure temperature and adjust via PID at 8 Hz
    }
    
    TCNT1 = 34285;
}
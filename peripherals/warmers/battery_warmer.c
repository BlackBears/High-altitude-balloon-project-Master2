

#include "battery_warmer.h"
#include <avr/io.h>
#include <util/delay.h>
#include "../../common/pindefs.h"

const uint8_t battery_warmer_profile[17][16] PROGMEM = 
{
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},  // 0
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},  // 1
    {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1},  // 2
    {0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,1},  // 3
    {0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1},  // 4
    {0,0,0,1,0,0,0,1,0,0,0,1,0,1,0,1},  // 5
    {0,0,0,1,0,1,0,1,0,0,0,1,0,1,0,1},  // 6
    {0,1,0,1,0,1,0,1,0,0,0,1,0,1,0,1},  // 7
    {0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},  // 8
    {0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,1},  // 9
    {0,1,0,1,0,1,0,1,1,1,0,1,0,1,1,1},  // 10
    {1,1,0,1,0,1,0,1,1,1,0,1,0,1,1,1},  // 11
    {1,1,0,1,0,1,0,1,1,1,1,1,0,1,1,1},  // 12
    {1,1,0,1,0,1,0,1,1,1,1,1,1,1,1,1},  // 13
    {1,1,0,1,1,1,0,1,1,1,1,1,1,1,1,1},  // 14
    {1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1},  // 15
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},  // 16
};

u08 bat_warmer_idx;
volatile u08 bat_warmer_val;

static void _battery_warmer_output(u08 value) {
    if( value ) {
        BAT_WARMER_PORT |= (1<<BAT_WARMER_PIN);
    }
    else {
        BAT_WARMER_PORT &= ~(1<<BAT_WARMER_PIN);
    }
}

void battery_warmer_setup(void) {
    DDR(BAT_WARMER_PORT) |= (1<<BAT_WARMER_PIN);    //  set as output
    _battery_warmer_output(0);                      //  set to off
    
    bat_warmer_idx = 0;
    bat_warmer_val = 0;
}

void battery_warmer_update(void) {
    u08 output;
    
    output = pgm_read_byte( &(battery_warmer_profile[bat_warmer_val][bat_warmer_idx]) );
    
    _battery_warmer_output(output);
    
    //  loop around
    if( bat_warmer_idx == 9 ) { bat_warmer_idx = 0; }
    else { bat_warmer_idx++; }
}

//  expects values in the range of 0 - 0xFF and shifts to 0x0F;
void battery_warmer_set(uint8_t value) {
    bat_warmer_var = value >> 4;
}
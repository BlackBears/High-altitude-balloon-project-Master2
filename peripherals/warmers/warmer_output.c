

#include "warmer_output.h"
#include "warmer.h"
#include "pindefs.h"

const u08 output_profile[17][8] PROGMEM =
{
    {0,0,0,0,0,0,0,0},  //  0
    {0,0,0,0,0,0,0,1},  //  1
    {0,0,0,1,0,0,0,1},  //  2
    {0,0,0,1,0,1,0,1},  //  3
    {0,1,0,1,0,1,0,1},  //  4
    {1,1,0,1,0,1,0,1},  //  5
    {1,1,0,1,1,1,0,1},  //  6
    {1,1,1,1,1,1,0,1},  //  7
    {1,1,1,1,1,1,1,1}   //  8
};

u08 output_idx;         //  points to the 1 or 0 on_off value in a power level
volatile output_value;  //  0xFF >> 4 = 0xF

/* FUNCTION PROTOTYPES */

/*  CODE    */
static void _warmer_output(volatile warmer_t *warmer, u08 on_off) {
    if( warmer->type == k_battery_warmer ) {
        if( on_off ) {
            BAT_WARMER_PORT |= (1<<BAT_WARMER_PIN);
        }
        else {
            BAT_WARMER_PORT &= ~(1<<BAT_WARMER_PIN);
        }
    }
}

void warmer_setup(void) {
    warmer[k_warmer_battery].pid.idx = 0;
    warmer[k_warmer_battery].pid.power = 0;
    _warmer_output(warmer[k_warmer_battery], 0);
    
    //  repeat this with each other warmer as we implement them...
}

void warmer_set(volatile warmer_t *warmer, u08 raw_power) {
    warmer[k_warmer_battery].pid.power = raw_power >> 4;
    
    //  repeat this with other warmers as we implement them...
}

//  update our controller output at 64 Hz
void warmer_update_64Hz(void) {
    //  update each of the warmers according to its current params
    
    //  get local vars for convenience
    u08 on_off_val;
    u08 scaled_power_level = warmer[k_warmer_battery].pid.power;
    u08 output_index = warmer[k_warmer_battery].pid.idx;
    
    u08 on_off_val = pgm_read_byte( &(output_profile[scaled_power_level][output_index]) );
    _warmer_output(warmers[k_warmer_battery], on_off_val);
    
    //  if we reached the end our index, loop
    if( output_index == 8 ) {
        warmers[k_warmer_battery].pid.idx = 0;
    }
    else {
        warmer[k_warmer_battery].pid.idx = ++output_index;
    }
    
}

//  take a temperature reading at 8 Hz and process w PID
void warmer_update_8Hz(void) {
    //  do this for each of our warmers
    warmer_read_temp(warmer[k_warmer_battery]);     //  read the current temperature
    
    u08 raw_power = warmer_pid_update(&warmer[k_warmer_battery]);
    warmer_set(&warmer[k_warmer_battery], raw_power);
    
}
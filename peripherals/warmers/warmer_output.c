

#include "warmer_output.h"
#include "warmer.h"
#include "../../common/pindefs.h"
#include <avr/io.h>
#include <avr/pgmspace.h>

#define WARMER_OUTPUT_DEBUG 1
#if WARMER_OUTPUT_DEBUG == 1
#include "../../capabilities/uart-644a.h"
char buffer[60];
u08 debug_count;
#endif

const u08 output_profile[8][17] PROGMEM =
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
#define WARMER_OUTPUT_DEBUG_DETAIL 0

static void _warmer_output(u08 idx, u08 on_off) {
    if( idx == WARMER_BATTERY ) {
        if( on_off ) {
            BAT_WARMER_PORT |= (1<<BAT_WARMER_PIN);
#if WARMER_OUTPUT_DEBUG_DETAIL == 1
	uart1_puts("*");
#endif
        }
        else {
            BAT_WARMER_PORT &= ~(1<<BAT_WARMER_PIN);
#if WARMER_OUTPUT_DEBUG_DETAIL == 1
	uart1_puts("_");
#endif
        }
    }
}

void warmer_setup(void) {
	
#if WARMER_OUTPUT_DEBUG == 1
	debug_count = 0;
#endif
    warmers[WARMER_BATTERY].pid.output.idx = 0;
    warmers[WARMER_BATTERY].pid.output.power = 0;
    _warmer_output(&warmers[WARMER_BATTERY], 0);
    
    //  repeat this with each other warmer as we implement them...
}

void warmer_set(volatile warmer_t *warmer, u08 raw_power) {
    warmer[WARMER_BATTERY].pid.output.power = raw_power >> 4;
    
    //  repeat this with other warmers as we implement them...
}

//  update our controller output at 64 Hz
void warmer_update_64Hz(void) {
    //  update each of the warmers according to its current params
    
    //  get local vars for convenience
    u08 scaled_power_level = warmers[WARMER_BATTERY].pid.output.power;
    u08 output_index = warmers[WARMER_BATTERY].pid.output.idx;
#if WARMER_OUTPUT_DEBUG == 1
	if( debug_count % 63 == 0) {
		//sprintf(buffer,"scaled_pwr = %02d | idx = %02d\r",scaled_power_level,output_index);
		//uart1_puts(buffer);	
	}		
#endif
    
    u08 on_off_val = pgm_read_byte( &(output_profile[scaled_power_level][output_index]) );
    _warmer_output(WARMER_BATTERY, on_off_val);
    
    //  if we reached the end our index, loop
    if( output_index == 8 ) {
        warmers[WARMER_BATTERY].pid.output.idx = 0;
    }
    else {
        warmers[WARMER_BATTERY].pid.output.idx = ++output_index;
    }
}

//  take a temperature reading at 8 Hz and process w PID
void warmer_update_8Hz(void) {
    //  do this for each of our warmers
    warmer_read_temp(WARMER_BATTERY);     //  read the current temperature
    u08 raw_power = warmer_pid_update(&warmers[WARMER_BATTERY]);
	warmer_set(&warmers[WARMER_BATTERY], raw_power);
#if WARMER_OUTPUT_DEBUG == 1
	debug_count++;
	if( debug_count % 63 == 0 ) {
		sprintf(buffer,"Warmer c_temp = %02d | ttemp = %02d | r_pwr = %02d s_pwr = %02d\r",warmers[WARMER_BATTERY].current_temp,warmers[WARMER_BATTERY].target_temp,raw_power,warmers[WARMER_BATTERY].pid.output.power);
		uart1_puts(buffer);
		debug_count = 0;
	}	
#endif
    
}
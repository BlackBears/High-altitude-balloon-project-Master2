

#include "warmer.h"
#include "../../capabilities/a2d.h"
#include "../../common/pindefs.h"
#include <avr/io.h>
#include <avr/pgmspace.h>

const u08 warmer_profile[8][7] PROGMEM =
{
    {0,0,0,0,0,0,0},    // 0
    {0,0,0,0,0,0,1},    // 1
    {0,0,1,0,0,0,1},    // 2
    {1,0,1,0,0,0,1},    // 3
    {1,0,1,0,1,0,1},    // 4
    {1,0,1,1,1,0,1},    // 5
    {1,1,1,1,1,0,1},    // 6
    {1,1,1,1,1,1,1},    // 7
};

/*  FUNCTION PROTOTYPES */

void _init_battery_warmer(void);
void _init_camera_warmer(volatile warmer_t *warmer);
void _init_cellular_warmer(volatile warmer_t *warmer);
void _warmer_battery_read_temp(void);
void _warmer_output(volatile warmer_t *warmer, u08 warmer_on_off);
void warmer_output_update(volatile warmer_t *warmer);
void warmer_output_setup(volatile warmer_t *warmer);
void warmer_output_set(volatile warmer_t *warmer,u08 cmd_value);

/*  CODE */

//
//  initialize the controller, primarily by setting up the ADC
//
void warmer_controller_init(void) {
    a2dInit();
    
    //  initialize the battery warmer
    _init_battery_warmer();
}


//
//  read the current temp of the warmer and store it in warmer->current_temp
//
void warmer_read_temp(u08 idx) {
	switch(idx) {
		case WARMER_BATTERY:
			return _warmer_battery_read_temp();
			break;
	}
}

void _warmer_battery_read_temp(void) {
	warmers[WARMER_BATTERY].current_temp = a2dConvert10bit(warmers[WARMER_BATTERY].adc_channel)/2;
}

/*  WARMER INITIALIZATION */

#warning ADC channel for battery warmer needs to be set according to schematic
void _init_battery_warmer(void) {
	warmers[WARMER_BATTERY].pid.k_p = 13;
	warmers[WARMER_BATTERY].pid.k_i = 3;
	warmers[WARMER_BATTERY].pid.k_d = 13;
	warmers[WARMER_BATTERY].pid.k_div = 8;
	warmers[WARMER_BATTERY].adc_channel = 0;
	warmers[WARMER_BATTERY].target_temp = 28;	//  28 = 28.0 C = 82.4 F 
    
    DDR(BAT_WARMER_PORT) |= (1<<BAT_WARMER_PIN);
    BAT_WARMER_PORT &= ~(1<<BAT_WARMER_PIN);
}

void _init_camera_warmer(volatile warmer_t *warmer) {
    warmer->pid.k_p = 13;
    warmer->pid.k_i = 3;
    warmer->pid.k_d = 13;
    warmer->pid.k_div = 8;
}

void _init_cellular_warmer(volatile warmer_t *warmer) {
    warmer->pid.k_p = 13;
    warmer->pid.k_i = 3;
    warmer->pid.k_d = 13;
    warmer->pid.k_div = 8;
}
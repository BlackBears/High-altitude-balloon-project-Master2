

#include "warmer.h"
#include "../../capabilities/a2d.h"
#include <avr/io.h>

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
}

/*  FUNCTION PROTOTYPES */

void _init_battery_warmer(volatile warmer_t *warmer);
void _init_camera_warmer(volatile warmer_t *warmer);
void _init_cellular_warmer(volatile warmer_t *warmer);
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
    _init_batter_warmer(warmer[k_warmer_battery]);
}


//
//  read the current temp of the warmer and store it in warmer->current_temp
//
void warmer_read_temp( volatile warmer_t *warmer) {
	warmer->current_temp = a2dConvert10bit(warmer->adc_channel)/2;
}

/*  WARMER INITIALIZATION */

void _init_battery_warmer(volatile warmer_t *warmer) {
    warmer->pid.k_p = 13;
    warmer->pid.k_i = 3;
    warmer->pid.k_d = 13;
    warmer->pid.k_div = 8;
    
    warmer->target_temp = 280;  //  280 = 28.0 C = 82.4 F
    
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


#ifndef __WARMER_H
#define __WARMER_H

#include "../../common/global.h"

#define k_delay 10
#define warmer_count 1

#define WARMER_BATTERY 0
#define WARMER_CAMERA 1
#define WARMER_CELL
typedef u08 warmer_type_t;

typedef struct {
    u08 idx;
    u08 power;
} warmer_pid_output_t;

typedef struct {
    u08 k_p;
    u08 k_i;
    u08 k_d;
    u08 k_div;
    s16 pid_prev[k_delay];		//  prev temperatures
    s32 pid_int;				//  integral
    u08 pid_prev_index;         //  previous temp index
    warmer_pid_output_t output;
} warmer_pid_t;

typedef struct {
    warmer_type_t type;
    u08 min_temp;
    u08 max_temp;
    s16 target_temp;
	s16 current_temp;
    u08 adc_channel;
    warmer_pid_t pid;       //  PID controller
} warmer_t;

volatile warmer_t warmers[1];

void warmer_controller_init(void);
void warmer_read_temp(u08 idx);
uint8_t warmer_pid_update(volatile warmer_t *warmer);

#endif
#ifndef __BATTERY_WARMER_PID_H
#define __BATTERY_WARMER_PID_H

#include "../common/global.h"

void pid_reset(void);
uint8_t pid_update(int16_t temp, int16_t target);

#endif


#ifndef __WARMER_PID_H
#define __WARMER_PID_H

#include "../../common/global.h"
#include "warmer.h"

//  returns raw power level in the range of 0x00 to 0xFF
u08 warmer_pid_update(volatile warmer_t *warmer);

void warmer_pid_reset(volatile warmer_t *warmer);

#endif
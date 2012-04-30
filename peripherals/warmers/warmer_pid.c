

#include "warmer_pid.h"

s16 _warmer_prev_update(volatile warmer_t *warmer, s16 prev) {
    //  local var for convenience
    u08 pid_prev_index = warmer->pid.pid_prev_index;
    
    //  pop last value off the stack
    int16_t popped = warmer->pid.pid_prev[pid_prev_index];
    warmer->pid.pid_prev[pid_prev_index] = prev;

    //  implement as ring buffer
    pid_prev_index++;
    if(pid_prev_index >= k_delay)
        pid_prev_index = 0;
    warmer->pid.pid_prev_index = pid_prev_index;

    return popped;
}

void warmer_pid_reset(volatile warmer_t *warmer) {
    for( u08 i = 0; i < k_delay; i++ ) {
        warmer->pid.pid_prev[i] = 25;  //  25.0 degrees C
    }
    warmer->pid.pid_int = 0;                //  integral is zero
    warmer->pid.pid_prev_index = 0;     //  ring buffer pointer is zero
}

u08 warmer_pid_update(volatile warmer_t *warmer) {
    s16 error, derivative;
    s32 command;
    
    //  calculate error terms
    s16 target = warmer->target_temp;
    s16 temp = warmer->current_temp;
    error = target - temp;      //  this is the current delta
    derivative = _warmer_prev_update(warmer,temp) - temp;  //  last delta
    
    //  sum the weighted terms
    command  = ((s32)error)                 << warmer->pid.k_p;
    command += ((s32)warmer->pid.pid_int)   << warmer->pid.k_i;
    command += ((s32)derivative)            << warmer->pid.k_d;
    
    //  scale the command
    command >>= warmer->pid.k_div;
    
    //  only update integral if output is not saturated
    if( (command >= 0 && command <= 0xFF) ||
        (command > 0 && error < 0) ||
        (command < 0 && error > 0) ) {
            warmer->pid.pid_int = warmer->pid.pid_int + error;
        }
    
    //  limit the range of the raw power command
    if( command < 0 ) { command = 0; }
    else if( command > 0xFF ) { command = 0xFF; }
    
    return (u08)command;
}


#ifndef __WARMER_H
#define __WARMER_H

#include "../../common/global.h"

enum {
    k_warmer_battery,
    k_warmer_camera,
    k_warmer_cellular,
};
typedef u08 warmer_type_t;

typedef struct {
    warmer_type_t type;
    u08 min_temp;
    u08 max_temp;
	u16 current_temp;
    u08 adc_channel;
} warmer_t;

void warmer_init(void);
void warmer_read_temp( volatile warmer_t *warmer);

#endif
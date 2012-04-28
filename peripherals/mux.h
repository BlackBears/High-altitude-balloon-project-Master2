

#ifndef __MUX_H
#define __MUX_H

#include "../common/global.h"

typedef enum {
    mux_open_log,
    mux_lcd,
    mux_terminal
} mux_channel_t;

void mux_init(void);
void mux_select_channel(mux_channel_t chan);

#endif
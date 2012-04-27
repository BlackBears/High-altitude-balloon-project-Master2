

#ifndef __MUX_H
#define __MUX_H

#include "../common/global.h"

enum {
    mux_open_log,
    mux_lcd,
    mux_terminal
};
typedef mux_channel_t;

void mux_init(void);
void mux_select_channel(mux_channel_t chan);

#endif
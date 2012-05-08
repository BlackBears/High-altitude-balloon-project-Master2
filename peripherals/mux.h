

#ifndef __MUX_H
#define __MUX_H

#include "../common/global.h"

typedef uint8_t mux_channel_t;
static const mux_channel_t MUX_OPEN_LOG = 0;
static const mux_channel_t MUX_TERMINAL = 1;

mux_channel_t mux_current_channel;

void mux_init(void);
void mux_select_channel(mux_channel_t chan);

#endif
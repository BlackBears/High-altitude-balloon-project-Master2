/*! \file dx.h  \brief Diagnostic indicator control
//*****************************************************************************
//
// Filename         :   dx.h
// Title            :   Diagnostic indicator control
// Author           :   Alan K Duncan <duncan.alan@mac.com>
// Created          :   2012-04-30 11-19-53
// Revised          :   2012-04-30 11-20-11
// Version          :   1.0
// Target MCU       |   Atmel AVR
//*****************************************************************************

/*-----------------------------------------------------------------------------
 * Copyright (c) 2012 Alan K Duncan >
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   - Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   - Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   - The name of the author may not be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
-----------------------------------------------------------------------------*/

#ifndef __DX_H
#define __DX_H

#include "../common/global.h"

#define DX_INDICATOR_COUNT 3

typedef struct {
    u08 count;      //  times to flash
    u32 last_m;     //  millisecond count at last transition
    BOOL state;     //  ON/OFF state
} dx_indicator_t;

//! flashes the indicator with given index
//  count gives the number of flashes
//  m gives the current milliseconds from the 1kHz interrupt
void dx_indicator_flash(u08 index,u08 count,u32 m);

//! initializes all of the diagnostic indicators
void dx_indicator_init(void);

//! this updates all of the indicators.
//  it should be called as often as necessary from the main loop
void dx_indicator_update(u32 m);

dx_indicator_t dx_indicators[DX_INDICATOR_COUNT];

#endif
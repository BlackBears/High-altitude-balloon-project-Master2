/*! \file dx.h  \brief Diagnostic indicator control
//*****************************************************************************
//
// Filename         :   dx.c
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


#include "dx.h"
#include <avr/io.h>
#include "../common/pindefs.h"

/*  FUNCTION PROTOTYPES */
void dx_indicator_set_state(u08 index, BOOL state);

void dx_indicator_init(void) {
	DDR(DX_1_PORT) |= (1<<DX_1_PIN);
    DDR(DX_2_PORT) |= (1<<DX_2_PIN);
    DDR(DX_3_PORT) |= (1<<DX_3_PIN);

    for(u08 i = 0; i < DX_INDICATOR_COUNT; i++) {
        dx_indicator_set_state(i,FALSE);
        dx_indicators[i].count = 0;
    }
}
void dx_indicator_flash(u08 index,u08 count,u32 m) {
    dx_indicator_set_state(index,TRUE);
    dx_indicators[index].count = count;
    dx_indicators[index].last_m = m;
}

void dx_indicator_update(u32 m) {
    for(u08 i = 0; i < DX_INDICATOR_COUNT; i++) {
        if( dx_indicators[i].count != 0 ) {
            if( m >= dx_indicators[i].last_m + 500 ) {
                if( dx_indicators[i].state ) {
                    dx_indicator_set_state(i,FALSE);
                    dx_indicators[i].count--;
                }
                else
                    dx_indicator_set_state(i,TRUE);
            }   //  transition time
        }
        else
            dx_indicator_set_state(i,FALSE);
    }   // indicators iteration
}

void dx_indicator_set_state(u08 index, BOOL state) {
    switch( index )
    {
        case 0:
            if( state )
                DX_1_PORT |= (1<<DX_1_PIN);
            else
                DX_1_PORT &= ~(1<<DX_1_PIN);
            break;
        case 1:
            if( state )
                DX_2_PORT |= (1<<DX_2_PIN);
            else
                DX_2_PORT &= ~(1<<DX_2_PIN);
            break;
        case 2:
            if( state )
                DX_3_PORT |= (1<<DX_3_PIN);
            else
                DX_3_PORT &= ~(1<<DX_3_PIN);
            break;
    }
    dx_indicators[index].state = state;
}
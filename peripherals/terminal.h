//////////////////////////////////////////////////////////////////////////////////////////
//	
//	File		: 'terminal.h'
//	Author		: Alan K. Duncan <duncan.alan@mac.com>
//	Created		: 2012-05-01
//	Revised		: 2012-05-09
//	Version		: 1.0
//	Target MCU	: ATmega644A
//	
//	This file provides terminal functionality for the HAB project.  In preflight, 
//	the user can attach a computer to the payload and interact with it for testing.
//	This also makes debugging much easier than printing debugging strings to an
//	LCD.
//	To use the terminal, it must be initialized with terminal_init.  Thereafter,
//	the main loop needs to take characters off UART1 and call terminal_process_char with
//	the returned character.
//
//////////////////////////////////////////////////////////////////////////////////////////


#ifndef __TERMINAL_H_
#define __TERMINAL_H_

#include "../common/global.h"

void terminal_init(void);
void terminal_process_char(char data);

#endif
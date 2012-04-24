/*
 * global.h
 *
 * Created: 4/2/2012 3:40:29 PM
 *  Author: Owner
 */ 


#ifndef GLOBAL_H_
#define GLOBAL_H_

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include "avrlibdefs.h"
#include "avrlibtypes.h"

/* address of data direction register of port x */

#define DDR(x) (*(&x - 1))   

#ifndef PIN
	#if defined(__AVR_ATmega64__) || defined(__AVR_ATmega128__)
		/* on ATmega64/128 PINF is on port 0x00 and not 0x60 */
		#define PIN(x) ( &PORTF==&(x) ? _SFR_IO8(0x00) : (*(&x - 2)) )
	#else
		#define PIN(x) (*(&x - 2))    /* address of input register of port x          */
	#endif
#endif

#define UART0 0
#define UART1 1

#endif /* GLOBAL_H_ */
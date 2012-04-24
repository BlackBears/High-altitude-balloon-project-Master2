/*
 * openlog.c
 *
 * Created: 4/24/2012 9:02:21 AM
 *  Author: Administrator
 */ 

#define BAUD 9600
#include <util/setbaud.h>
#include "openlog.h"
#include "../common/pindefs.h"
#include "../capabilities/uart.h"
#include <avr/io.h>
#include <util/delay.h>

#define ASCII_SUB 0x1A

void open_log_init(void) {
	int baud = (UBRRH_VALUE << 8) | UBRRL_VALUE;
#if OPEN_LOG_UART == UART0
	uart_init(baud);
#else
	uart1_init(baud);
#endif
	OPEN_LOG_PWR_PORT &= ~(1<<OPEN_LOG_PWR_PIN);
	DDR(OPEN_LOG_PWR_PORT) |= (1<<OPEN_LOG_PWR_PIN);
	
	//	setup our reset port and pin, hold reset high
	OPEN_LOG_RESET_PORT |= (1<<OPEN_LOG_RESET_PIN);
	DDR(OPEN_LOG_RESET_PORT) |= (1<<OPEN_LOG_RESET_PIN);
}

void open_log_set_pwr(BOOL pwr_state) {
	if( pwr_state ) {
		OPEN_LOG_PWR_PORT |= (1<<OPEN_LOG_PWR_PIN);
	}
	else {
		OPEN_LOG_PWR_PORT &= ~(1<<OPEN_LOG_PWR_PIN);
	}
}

void open_log_write_test(void) {
	open_log_reset();
	open_log_command_mode();
	char ack;
	#if OPEN_LOG_UART == UART0
		uart_puts("new temps.txt\r");
		while(1) {
			ack = uart_getc();
			if( ack == '>' ) { break; }
		}
		uart_puts("append temps.txt\r");
		uart_puts("Hello World!\r");
	#else
		uart1_puts("new temps.txt\r");
		while(1) {
			ack = uart1_getc();
			if( ack == '>' ) { break; }
		}
		uart1_puts("append temps.txt\r");
		uart1_puts("Hello World!\r");
	#endif
	
}

void open_log_command_mode() {
	#if OPEN_LOG_UART == UART0
		uart_putc(ASCII_SUB);
		uart_putc(ASCII_SUB);
		uart_putc(ASCII_SUB);
		char ack;
		while(1) {
			ack = uart_getc();
			if( ack == '>' ) { break; }
		}
	#else	
		uart1_putc(ASCII_SUB);
		uart1_putc(ASCII_SUB);
		uart1_putc(ASCII_SUB);
		while(1) {
			ack = uart1_getc();
			if( ack == '>' ) { break; }
		}
	#endif
}

void open_log_reset() {
	OPEN_LOG_RESET_PORT &= ~(1<<OPEN_LOG_RESET_PIN);
	_delay_ms(100);
	OPEN_LOG_RESET_PORT |= (1<<OPEN_LOG_RESET_PIN);
	//	wait for the device to acknowledge before returning
	while(1) {
		char ack;
		#if OPEN_LOG_UART == UART0
			ack = uart_getc();
		#else
			ack = uart1_getc();
		#endif
		if( ack == '<' ) { break; }
	}
}
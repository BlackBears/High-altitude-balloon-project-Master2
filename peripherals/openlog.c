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
#include "../capabilities/vfd.h"
#include <avr/io.h>
#include <util/delay.h>

#define ASCII_SUB 0x1A

void _open_log_wait_cmd_ack();
void _open_log_wait_log_ack();

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
	vfd_cls();	vfd_puts("Will go cmd");
	_delay_ms(1000);
	open_log_command_mode();				//	put in command mode and wait until ack
	vfd_cls(); vfd_puts("Finished cmd");
	_delay_ms(1000);
	#if OPEN_LOG_UART == UART0
		uart_puts("rm temp01.txt");
		_delay_ms(10);
		//_open_log_wait_cmd_ack();
		vfd_cls();	vfd_puts("Removed old temp01");
		_delay_ms(1000);
		uart_puts("new temps01.txt\r");		//	create new file
		//_open_log_wait_cmd_ack();			//	for cmd (">") acknowledgement
		_delay_ms(10);
		vfd_cls();	vfd_puts("Created new temp01");
		_delay_ms(1000);
		uart_puts("append temps01.txt\r");
		//_open_log_wait_log_ack();			//	for log ("<") acknowledgement
		_delay_ms(10);
		uart_puts("Hello World!\r");		//	log something
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
		_delay_ms(10);
		//_open_log_wait_cmd_ack();
#else	
		uart1_putc(ASCII_SUB);
		uart1_putc(ASCII_SUB);
		uart1_putc(ASCII_SUB);
		_open_log_wait_cmd_ack();
#endif
}

void _open_log_wait_cmd_ack() {
	char ack;
	while(1) {
#if OPEN_LOG_UART == UART0
		ack = uart_getc();
#else
		ack = uart1_getc();
#endif
		if( ack == '>' ) { break; }
	}
}

void _open_log_wait_log_ack() {
	char ack;
	while(1) {
		#if OPEN_LOG_UART == UART0
		ack = uart_getc();
		#else
		ack = uart1_getc();
		#endif
		if( ack == '<' ) { break; }
	}
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


#include "terminal.h"
#include "../capabilities/uart.h"
#include "../capabilities/a2d.h"
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>


#define MAX_TERM_BUFFER_LEN 15
#define CLEAR_RX_BUFFER term_buffer[0] = '\0'

char term_buffer[MAX_TERM_BUFFER_LEN];
char out_buffer[20];

extern s16 get_internal_temperature();  //  implemented in main program file
extern s16 get_external_temperature();  //  implemented in main program file
extern long barometric_pressure();  //  implemented in main program file
extern u08 get_humidity();              //  implemented in main program file
extern BOOL internal_temperature_power();   //  implemented in main pgm file
extern BOOL external_temperature_power();   //  implemented in main pgm file
extern void set_internal_temperature_power(BOOL status);    //  in main pgm
extern void set_external_temperature_power(BOOL status);    //  in main pgm

/*  FUNCTION PROTOTYPES */

void _terminal_print_prompt(void);
void _terminal_print_welcome(void);
void _terminal_print_status(BOOL status);

void terminal_init(void) {
	uart1_putc(0x0C);
	_terminal_print_welcome();
	_terminal_print_prompt();
}

void terminal_process_char(char data) {
	//sprintf(out_buffer,"%s\r",term_buffer); uart1_puts(out_buffer);
    if( data == 0x0D ) {
        if( strcmp_P(term_buffer,PSTR("AT")) == 0 ) {
            uart1_puts_P("\rOK\r");
            _terminal_print_prompt();
			CLEAR_RX_BUFFER;
        }
        else if( strcmp_P(term_buffer,PSTR("AT+V?")) == 0 ) {
            uart1_puts_P("\r0.9.0\r");
            _terminal_print_prompt();
			CLEAR_RX_BUFFER;
        }
        else if( strcmp_P(term_buffer,PSTR("AT+IT?")) == 0 ) {
            //  read internal temperature
            s16 t = get_internal_temperature();
            sprintf(out_buffer,"\r%02dC\r",t);
            uart1_puts(out_buffer);
            _terminal_print_prompt();
			CLEAR_RX_BUFFER;
        }
        else if( strcmp_P(term_buffer,PSTR("AT+ET?")) == 0 ) {
            //  read external temperature
            s16 t = get_external_temperature();
            sprintf(out_buffer,"\r%02dC\r",t);
            uart1_puts(out_buffer);
            _terminal_print_prompt();
			CLEAR_RX_BUFFER;
        }
        else if( strcmp_P(term_buffer,PSTR("AT+BP?")) == 0 ) {
            long p = barometric_pressure();
            sprintf(out_buffer,"\r%06ld Pa\r",p);
            uart1_puts(out_buffer);
            _terminal_print_prompt();
			CLEAR_RX_BUFFER;
        }
        else if( strcmp_P(term_buffer,PSTR("AT+H?")) == 0 ) {
            u08 rh = humidity();
            sprintf(out_buffer,"\r%02d\r",rh);
            uart1_puts(out_buffer);
            _terminal_print_prompt();
			CLEAR_RX_BUFFER;
        }
        else if( strcmp_P(term_buffer,PSTR("AT+VOLT?")) == 0 ) {
			//
			// see: http://www.ikalogic.com/avr-monitor-power-supply-voltage-for-free/
			//
            ADMUX = 0x0E;   //  band gap voltage is the ADC input
            uint8_t adc_data = a2dConvert8bit(ADC_CH_122V);
			a2dInit();
			
			float vcc = 1230.0 * 255/adc_data;
			uint16_t vcc_i = (uint16_t)vcc;
			sprintf(out_buffer,"\r%d\r",vcc_i);
			uart1_puts(out_buffer);
			_terminal_print_prompt();
			CLEAR_RX_BUFFER;
        }
        else if( strcmp_P(term_buffer,PSTR("AT+ITP?")) == 0 ) {
            BOOL status = internal_temperature_power();
            _terminal_print_status(status);
        }
        else if( strcmp_P(term_buffer,PSTR("AT+ITP=")) > 0) {
            char c_state = term_buffer[7];
            uint8_t i_status = atoi(&term_buffer[7]);
            if( status == 0 ) {
                set_internal_temperature_power(FALSE);
                _terminal_print_status(FALSE);
            }
            else if( status == 1 ) {
                set_internal_temperature_power(TRUE);
                _terminal_print_status(TRUE);
            }
            else {
                CLEAR_RX_BUFFER;
			    uart1_puts_P("\rERROR 3\r");
            }
        }
        else if( strcmp_P(term_buffer,PSTR("AT+ETP?")) == 0 ) {
            BOOL status = external_temperature_power();
            _terminal_print_status(status);
        }
        else if( strcmp_P(term_buffer,PSTR("AT+ETP=")) > 0) {
            char c_state = term_buffer[7];
            uint8_t i_status = atoi(&term_buffer[7]);
            if( status == 0 ) {
                set_external_temperature_power(FALSE);
                _terminal_print_status(FALSE);
            }
            else if( status == 1 ) {
                set_external_temperature_power(TRUE);
                _terminal_print_status(TRUE);
            }
            else {
                CLEAR_RX_BUFFER;
			    uart1_puts_P("\rERROR 3\r");
            }
        }
        
		else {
			CLEAR_RX_BUFFER;
			uart1_puts_P("\rERROR 2\r");
		}			
    }
    else if( strlen(term_buffer) == MAX_TERM_BUFFER_LEN -1 ) {
        //  error, impending overflow
        CLEAR_RX_BUFFER;
        uart1_puts_P("\rERROR 1\r");    // long text
    }
    else {			
        int len = strlen(term_buffer);
		term_buffer[len] = toupper(data);
		term_buffer[len + 1] = '\0';
    }
}

//
//  prints the standard prompt to the terminal
//
void _terminal_print_prompt(void) {
    uart1_puts_P("HAB>");
}

//
//  prints status of bool condition in plain english
//
void _terminal_print_status(BOOL status) {
    if( status )
        uart1_puts_P("\rON\r");
    else
        uart1_puts_P("\rOFF\r");
}

//  
//  prints a standard welcome message to the terminal
//
void _terminal_print_welcome(void) {
    uart1_puts_P("Welcome to HAB: High-altitude balloon controller\r");
    uart1_puts_P("Copyright (c) 2012 Alan K Duncan\r");
    uart1_puts_P("Software version 0.9.0\r\r");
}
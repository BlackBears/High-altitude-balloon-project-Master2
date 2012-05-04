

#include "terminal.h"
#include "../capabilities/uart-644a.h"
#include "../capabilities/a2d.h"
#include "../common/states.h"
#include "../common/types.h"
#include "../peripherals/mux.h"
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>


#define MAX_TERM_BUFFER_LEN 22
#define CLEAR_RX_BUFFER term_buffer[0] = '\0'

char term_buffer[MAX_TERM_BUFFER_LEN];
char out_buffer[22];

extern s16 get_internal_temperature();  //  implemented in main program file
extern s16 get_external_temperature();  //  implemented in main program file
extern long barometric_pressure();  //  implemented in main program file
extern u08 get_humidity();              //  implemented in main program file
extern BOOL internal_temperature_power();   //  implemented in main pgm file
extern BOOL external_temperature_power();   //  implemented in main pgm file
extern void set_internal_temperature_power(BOOL status);    //  in main pgm
extern void set_external_temperature_power(BOOL status);    //  in main pgm
extern void rtc_read_time(time_t *time);
extern void rtc_set_time(time_t *time);

uint16_t terminal_bandgap_voltage(void);

//  
//  prints a standard welcome message to the terminal
//
static void _terminal_print_welcome(void) {
    uart1_puts_P("Welcome to HAB: High-altitude balloon controller\r");
    uart1_puts_P("Copyright (c) 2012 Alan K Duncan\r");
    uart1_puts_P("Software version 0.9.0\r\r");
}

//
//  prints the standard prompt to the terminal
//
static void _terminal_print_prompt(void) {
    uart1_puts_P("HAB>");
}

//	
//	print an error message with code
//	
static void _terminal_print_error(u08 error_num) {
	CLEAR_RX_BUFFER;
	uart1_puts_P("\rERROR ");
	sprintf(out_buffer,"%d\r",error_num);
	uart1_puts(out_buffer);
	_terminal_print_prompt();
}

//	
//	initializes the terminal, clearing it and printing a welcome message
//
void terminal_init(void) {
	uart1_putc(0x0C);
	_terminal_print_welcome();
	_terminal_print_prompt();
}

//
//  prints status of bool condition in plain English
//
static void _terminal_print_status(BOOL status) {
    if( status )
        uart1_puts_P("\rON\r");
    else
        uart1_puts_P("\rOFF\r");
}

//	
//	reads the bandgap voltage on the AVR to estimate its VCC
//	uses oversampling to reduce noise.
//
static uint16_t terminal_avg_bandgap_voltage(void) {
	uint16_t accumulator = 0;
	for(uint8_t i = 0; i < 63; i++) {
		accumulator += terminal_bandgap_voltage();
	}
	return (accumulator>>6);
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
			/*
			a2dInit();
			uint16_t adc_data = a2dConvert10bit(1);
			uart1_puts("Did a conversion");
			_delay_ms(2000);
			adc_data = a2dConvert10bit(1);
			*/
			a2dOff();
			ADMUX = (1<<REFS0);		// Use Vcc as the analog reference;
			ADCSRA |= (1<<ADEN);	// turn on the ADC circuitry
			ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // prescaler /128
			ADMUX |= (1<<MUX4) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1);
			ADMUX &= ~(1<<MUX0);
			
			uint16_t adc_data = terminal_avg_bandgap_voltage();
			
			
			float vcc = 1100.0f * (1023.0f/(float)adc_data);
			sprintf(out_buffer,"\rADC = %d (~ %0.0f mV)\r",adc_data,vcc);
			uart1_puts(out_buffer);
			_terminal_print_prompt();
			CLEAR_RX_BUFFER;
        }
		else if( strstr(term_buffer,"AT+RTC") ) {
			//	command is something related to RTC
			if( strstr(term_buffer,"?") ) {
				//	user is asking for the current time
				time_t current_time;
				rtc_read_time(&current_time);
				sprintf(out_buffer,"\r%02d:%02d:%02d\r", current_time.hour,current_time.minute,current_time.second);
				uart1_puts(out_buffer);
				_terminal_print_prompt();	
				CLEAR_RX_BUFFER;
			}
			else {
				char *eq_ptr = strstr(term_buffer,"=");
				if( eq_ptr ) {
					char *component_str = (char*)malloc(3);
					strncpy(component_str,term_buffer+7,2);
					component_str[3] = '\0';
					u08 hour = atoi(component_str);
					strncpy(component_str,term_buffer+10,2);
					component_str[3] = '\0';
					u08 minute = atoi(component_str);
					strncpy(component_str,term_buffer+13,2);
					component_str[3] = '\0';
					u08 second = atoi(component_str);
					free(component_str);
					time_t time = {.hour = hour, .minute = minute, .second = second, .new_second = FALSE};
					rtc_set_time(&time);
					uart1_puts_P("\rRTC set: ");
					sprintf(out_buffer,"%02d:%02d:%02d\r",hour,minute,second);
					uart1_puts(out_buffer);
					_terminal_print_prompt();
					CLEAR_RX_BUFFER;
				}
				
				else {
					_terminal_print_error(3);	// incorrect syntax	
				}
			}
		}
		else if( strstr_P(term_buffer,PSTR("AT+LTST")) ) {
			open_log_write_test();
			mux_select_channel(MUX_TERMINAL);
			_terminal_print_prompt();
			CLEAR_RX_BUFFER;
		}
		else {
			sprintf(out_buffer,"\rcmd = %s | len = %d\r",term_buffer,strlen(term_buffer));
			uart1_puts(out_buffer);
			_terminal_print_error(2);
		}			
    }
    else if( strlen(term_buffer) == MAX_TERM_BUFFER_LEN -1 ) {
        //  error, impending overflow
        _terminal_print_error(1);
    }
    else {			
        int len = strlen(term_buffer);
		term_buffer[len] = toupper(data);
		term_buffer[len + 1] = '\0';
    }
}


uint16_t terminal_bandgap_voltage(void) {
	//Start Single conversion
	ADCSRA|=(1<<ADSC);

	//Wait for conversion to complete
	while(!(ADCSRA & (1<<ADIF)));

	//Clear ADIF by writing one to it
	//Note you may be wondering why we have write one to clear it
	//This is standard way of clearing bits in io as said in datasheets.
	//The code writes '1' but it result in setting bit to '0' !!!

	ADCSRA|=(1<<ADIF);
	return (ADC);
}


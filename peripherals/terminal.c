

#include "terminal.h"
#include "../capabilities/uart2.h"
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
extern void open_log_ls(char *buffer, size_t size);
extern void set_serial_channel(mux_channel_t chan);
extern void set_ignore_serial_data(BOOL state);

uint16_t terminal_bandgap_voltage(void);

//  
//  prints a standard welcome message to the terminal
//
static void _terminal_print_welcome(void) {
    uartSendString_P(1,"Welcome to HAB: High-altitude balloon controller\r");
    uartSendString_P(1,"Copyright (c) 2012 Alan K Duncan\r");
    uartSendString_P(1,"Software version 0.9.0\r\r");
}

//
//  prints the standard prompt to the terminal
//
static void _terminal_print_prompt(void) {
    uartSendString_P(1,"HAB>");
}

//	
//	print an error message with code
//	
static void _terminal_print_error(u08 error_num) {
	CLEAR_RX_BUFFER;
	uartSendString_P(1,"\rERROR ");
	char out_buffer[15];
	sprintf(out_buffer,"%d\r",error_num);
	uartSendString(1,out_buffer);
	_terminal_print_prompt();
}

//	
//	initializes the terminal, clearing it and printing a welcome message
//
void terminal_init(void) {
	uartSendByte(1,0x0C);
	_terminal_print_welcome();
	_terminal_print_prompt();
}

//
//  prints status of bool condition in plain English
//
static void _terminal_print_status(BOOL status) {
    if( status )
        uartSendString_P(1,"\rON\r");
    else
        uartSendString_P(1,"\rOFF\r");
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
	char out_buffer[20];
	//sprintf(out_buffer,"%s\r",term_buffer); uartSendString(1,out_buffer);
    if( data == 0x0D ) {
        if( strcmp_P(term_buffer,PSTR("AT")) == 0 ) {
            uartSendString_P(1,"\rOK\r");
            _terminal_print_prompt();
			CLEAR_RX_BUFFER;
        }
        else if( strcmp_P(term_buffer,PSTR("AT+V?")) == 0 ) {
            uartSendString_P(1,"\r0.9.0\r");
            _terminal_print_prompt();
			CLEAR_RX_BUFFER;
        }
        else if( strcmp_P(term_buffer,PSTR("AT+IT?")) == 0 ) {
            //  read internal temperature
            s16 t = get_internal_temperature();
            sprintf(out_buffer,"\r%02dC\r",t);
            uartSendString(1,out_buffer);
            _terminal_print_prompt();
			CLEAR_RX_BUFFER;
        }
        else if( strcmp_P(term_buffer,PSTR("AT+ET?")) == 0 ) {
            //  read external temperature
            s16 t = get_external_temperature();
            sprintf(out_buffer,"\r%02dC\r",t);
            uartSendString(1,out_buffer);
            _terminal_print_prompt();
			CLEAR_RX_BUFFER;
        }
        else if( strcmp_P(term_buffer,PSTR("AT+BP?")) == 0 ) {
            long p = barometric_pressure();
            sprintf(out_buffer,"\r%06ld Pa\r",p);
            uartSendString(1,out_buffer);
            _terminal_print_prompt();
			CLEAR_RX_BUFFER;
        }
        else if( strcmp_P(term_buffer,PSTR("AT+H?")) == 0 ) {
            u08 rh = get_humidity();
            sprintf(out_buffer,"\r%02d\r",rh);
            uartSendString(1,out_buffer);
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
			uartSendString(1,"Did a conversion");
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
			uartSendString(1,out_buffer);
			_terminal_print_prompt();
			CLEAR_RX_BUFFER;
        }
		else if( strstr_P(term_buffer,PSTR("AT+V33?"))) {
			a2dInit();
			uint16_t adc_data = a2dConvert10bit(6);
			float vcc33 = 5000.0f * (float)adc_data/1023.0f;
			sprintf(out_buffer,"\r~ %0.2f mV\r",vcc33);
			uartSendString(1,out_buffer);
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
				uartSendString(1,out_buffer);
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
					uartSendString_P(1,"\rRTC set: ");
					sprintf(out_buffer,"%02d:%02d:%02d\r",hour,minute,second);
					uartSendString(1,out_buffer);
					_terminal_print_prompt();
					CLEAR_RX_BUFFER;
				}
				
				else {
					_terminal_print_error(3);	// incorrect syntax	
				}
			}
		}
		else if( strstr(term_buffer,"AT+LGCM") ) {
			BOOL success = FALSE;
			CLEAR_RX_BUFFER;
			set_ignore_serial_data(TRUE);		//	don't process serial input as terminal commands
			set_serial_channel(MUX_OPEN_LOG);	//	switch serial path to logger
			uartSendByte(1,0x1A);					//	control-z x 3 puts us in command mode
			uartSendByte(1,0x1A);
			uartSendByte(1,0x1A);
			for(u16 timeout = 0; timeout < 1000; timeout++) {
				u08 c = uart1GetByte();
				if( c == '>' ) {
					success = TRUE;
					break;
				}
				_delay_ms(1);
			}
			uartFlushReceiveBuffer(1);
			set_serial_channel(MUX_TERMINAL);
			if( success ) 
				uartSendString_P(1,"\r+ CMD mode\r");
			else
				uartSendString_P(1,"\r- CMD mode\r");
			set_ignore_serial_data(FALSE);
			_terminal_print_prompt();
			CLEAR_RX_BUFFER;
		}	//	LOG CMD
		else if( strstr(term_buffer,"AT+LGLS")) {
			CLEAR_RX_BUFFER;
			set_ignore_serial_data(TRUE);		//	don't process serial input as terminal commands
			set_serial_channel(MUX_OPEN_LOG);	//	switch serial path to logger
			uartSendString(1,"This is a test\r");
			_delay_ms(100);
			set_serial_channel(MUX_TERMINAL);
			set_ignore_serial_data(FALSE);
			_terminal_print_prompt();
			CLEAR_RX_BUFFER;
		}
		else if( strstr(term_buffer,"AT?")) {
			uartSendByte(1,'\r');
			uartSendString_P(1,"HAB TERMINAL COMMANDS\r\r");
			uartSendString_P(1,"AT+V?\t\t\tReturns the estimated +5V bus voltage\r");
			uartSendString_P(1,"AT+V33?\t\t\tReturns the estimated +3.3V bus voltage\r");
			uartSendString_P(1,"AT+IT?\t\t\tReturns the capsule interior temperature\r");
			uartSendString_P(1,"AT+ET?\t\t\tReturns the capsule exterior temperature\r");
			uartSendString_P(1,"AT+BP?\t\t\tReturns the barometric pressure in Pascals (Pa)\r");
			uartSendString_P(1,"AT+RTC?\t\t\tReads the real-time clock (RTC)\r");
			uartSendString_P(1,"AT+RTC=HH:MM:SS\t\tSets the RTC\r");
			uartSendString_P(1,"AT+LGCM\t\t\tPuts the logger into command mode\r");
			uartSendString_P(1,"AT+LGLS\t\t\tWrites a test string to open log\r");
			CLEAR_RX_BUFFER;
			_terminal_print_prompt();
		}
		else {
			sprintf(out_buffer,"\rcmd = %s | len = %d\r",term_buffer,strlen(term_buffer));
			uartSendString(1,out_buffer);
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


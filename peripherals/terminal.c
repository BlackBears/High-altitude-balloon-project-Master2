

#include "terminal.h"
#include "../capabilities/uart.h"
#include "../capabilities/a2d.h"
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>


#define MAX_TERM_BUFFER_LEN 15

char term_buffer[MAX_TERM_BUFFER_LEN];
char out_buffer[20];

extern s16 internal_temperature();  //  implemented in main program file
extern s16 external_temperature();  //  implemented in main program file
extern long barometric_pressure();  //  implemented in main program file
extern u08 humidity();              //  implemented in main program file

/*  FUNCTION PROTOTYPES */

void _terminal_print_prompt(void);

void terminal_process_char(char data) {
    if( term_buffer == 0x0D ) {
        if( strcmp_P(term_buffer,PSTR("AT")) == 0 ) {
            uart1_puts_P("OK\r");
            _terminal_print_prompt();
        }
        else if( strcmp_P(term_buffer,PSTR("AT+V?")) ) {
            uart1_puts_P("0.9.0\r");
            _terminal_print_prompt();
        }
        else if( strcmp_P(term_buffer,PSTR("AT+IT?")) ) {
            //  read internal temperature
            s16 t = internal_temperature();
            sprintf(outbuffer,"%02dC\r",s16);
            uart1_puts(outbuffer);
            _terminal_print_prompt();
        }
        else if( strcmp_P(term_buffer,PSTR("AT+ET?")) ) {
            //  read external temperature
            s16 t = external_temperature();
            sprintf(outbuffer,"%02dC\r",s16);
            uart1_puts(outbuffer);
            _terminal_print_prompt();
        }
        else if( strcmp_P(term_buffer,PSTR("AT+BP?")) ) {
            long p = barometric_pressure();
            sprintf(outbuffer,"%06d Pa\r",p);
            uart1_puts(outbuffer);
            _terminal_print_prompt();
        }
        else if( strcmp_P(term_buffer,PSTR("AT+H?")) ) {
            u08 rh = humidity();
            sprintf(outbuffer,"%02d\r",rh);
            uart1_puts(outbuffer);
            _terminal_print_prompt();
        }
        else if( strcmp_P(term_buffer,PSTR("AT+VOLT?")) ) {
            ADMUX = 0x0E;   //  band gap voltage is the ADC input
            ADCSRA = (1<<ADEN) | (1<<ADATE) | (1<<ADIE) | (1<<ADSC) | 5;
        }
    }
    else if( strlen(buffer) == MAX_TERM_BUFFER_LEN -1 ) {
        //  error, impending overflow
        term_buffer = "\0";
        uart1_puts_P("ERROR 1");    // long text
    }
    else {
        strcat(term_buffer,data);
    }
}

//
// see: http://www.ikalogic.com/avr-monitor-power-supply-voltage-for-free/
//
ISR(ADC_vect) {
    uint8_t adc_data = ADC>>2;
    a2dInit();
    sei();
    float vcc = 1230 * 255/adc_data;
    sprintf(outbuffer,"%4.0f mV\r",vcc);
    uart1_puts(outbuffer);
    _terminal_print_prompt();
}

void _terminal_print_prompt(void) {
    uart1_puts_P("HAB>");
}

void _terminal_print_welcome(void) {
    uart1_puts_P("Welcome to HAB: High-altitude balloon controller\r");
    uart1_puts_P("Copyright (c) 2012 Alan K Duncan\r");
    uart1_puts_P("Software version 0.9.0\r\r");
}
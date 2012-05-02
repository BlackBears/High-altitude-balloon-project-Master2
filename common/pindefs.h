/*
 * pindefs.h
 *
 * Created: 4/15/2012 6:17:20 AM
 *  Author: Administrator
 */ 


#ifndef PINDEFS_H_
#define PINDEFS_H_

/*  POWER PINS  */
#define PWR_CTL_PORT        PORTA
#define BMP085_PWR_PORT     PWR_CTL_PORT
#define BMP085_PWR_PIN      3
#define EXT_TEMP_PWR_PORT   PWR_CTL_PORT
#define EXT_TEMP_PWR_PIN    5
#define INT_TEMP_PWR_PORT   PWR_CTL_PORT
#define INT_TEMP_PWR_PIN    4
#define ACCEL_PWR_PORT      PWR_CTL_PORT
#define ACCEL_PWR_PIN       7
#define GPS_PWR_PORT        PWR_CTL_PORT
#define GPS_PWR_PIN         6

/*  DX INDICATORS */
#define DX_INDICATOR_PORT   PORTB
#define DX_1_PORT           DX_INDICATOR_PORT
#define DX_1_PIN            4
#define DX_2_PORT           DX_INDICATOR_PORT
#define DX_2_PIN            5
#define DX_3_PORT           DX_INDICATOR_PORT
#define DX_3_PIN            6
#define DX_4_PORT           DX_INDICATOR_PORT
#define DX_4_PIN            7

/*  SERIAL MULTIPLEXER */
#define SERIAL_MUX_PORT     PORTD
#define SERIAL_MUX_A_PORT   SERIAL_MUX_PORT
#define SERIAL_MUX_A_PIN    4
#define SERIAL_MUX_B_PORT   SERIAL_MUX_PORT
#define SERIAL_MUX_B_PIN    5

/*	BMP085 temperature and pressure sensor */

#define BMP085_EOC_PORT	PORTC
#define BMP085_EOC_PIN	PC0

/*	OPEN LOG	*/
#warning OL power port and pin are no needed now
#define OPEN_LOG_CONTROL_PORT	PORTD
#define OPEN_LOG_PWR_PORT		OPEN_LOG_CONTROL_PORT
#define OPEN_LOG_PWR_PIN		7
#define OPEN_LOG_RESET_PORT		OPEN_LOG_CONTROL_PORT
#define OPEN_LOG_RESET_PIN		6

/*  CUTDOWN CONTROLLER */
#warning These need to be confirmed
#define CUTDOWN_CONTROL_PORT    PORTD
#define CUTDOWN_CONTROL_PIN     4

/*  BATTERY WARMER */
#warning These need to be confirmed
#define BAT_WARMER_PORT         PORTB
#define BAT_WARMER_PIN          3

#define RTC_1HZ_INT	7	//	the RTC 1Hz signal will interrupt on INT7


#endif /* PINDEFS_H_ */
/*
 * pindefs.h
 *
 * Created: 4/15/2012 6:17:20 AM
 *  Author: Administrator
 */ 


#ifndef PINDEFS_H_
#define PINDEFS_H_

/*	BMP085 temperature and pressure sensor */

#define BMP085_EOC_PORT	PORTC
#define BMP085_EOC_PIN	PC0

/*	OPEN LOG	*/
#define OPEN_LOG_CONTROL_PORT	PORTD
#define OPEN_LOG_PWR_PORT		OPEN_LOG_CONTROL_PORT
#define OPEN_LOG_PWR_PIN		6
#define OPEN_LOG_RESET_PORT		OPEN_LOG_CONTROL_PORT
#define OPEN_LOG_RESET_PIN		5

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
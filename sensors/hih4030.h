/*
 * hih4030.h
 *
 * Created: 4/28/2012 11:17:48 PM
 *  Author: Administrator
 */ 


#ifndef HIH4030_H_
#define HIH4030_H_

#include "../common/global.h"

#define HIH4030_ADC_CHANNEL 7

void hih4030_init(void);
u08 hih4030_sensor_rh(void);
u08 hih4030_compensated_rh(s16 temperature);



#endif /* HIH4030_H_ */
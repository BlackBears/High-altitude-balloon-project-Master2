

#ifndef __CELLULAR_H_
#define __CELLULAR_H_

#include "../common/global.h"
#include "../common/types.h"
//
//	send the current altitude in meters to the cellular controller
//
void cellular_set_altitude(int32_t a);

//
//	send the current time as an rtc struct to the cellular controller
//
void cellular_set_time(time_t t);

//
//	send the current capsule temperature in degrees C to the cellular controller
//
void cellular_set_capsule_temp(int16_t t);

//
//	send the current bus voltage in millivolts to the cellular controller
//
void cellular_set_bus_voltage(uint16_t v);


#endif
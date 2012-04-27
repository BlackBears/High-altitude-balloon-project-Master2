/*
 * bmp085.h
 *
 * Created: 4/13/2012 11:14:00 PM
 *  Author: Administrator
 */ 


#ifndef BMP085_H_
#define BMP085_H_

#include "../common/global.h"
#include "../common/states.h"

typedef struct {
    long temperature;
    long pressure;
    BOOL is_valid;
    sensor_status_t status;
} bmp085_t;

void bmp085_pwr_set(bmp085_t *device, BOOL pwr_state);
void bmp085_init(bmp085_t *device);
void bmp085_convert(bmp085_t *device);

#endif /* BMP085_H_ */
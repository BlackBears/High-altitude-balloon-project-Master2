/*
 * gps.h
 *
 * Created: 5/8/2012 9:30:34 AM
 *  Author: Administrator
 */ 


#ifndef GPS_H_
#define GPS_H_

#include "../common/global.h"
#include "../common/types.h"
#include "../capabilities/buffer.h"

gps_info_t gps_info;
cBuffer *gpsBuffer;

void gps_init();
void gps_add_char(u08 data);

#endif /* GPS_H_ */
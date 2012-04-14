/*
 * ds1307.h
 *
 * Created: 3/30/2012 11:06:48 AM
 *  Author: Owner
 */ 


#ifndef DS1307_H_
#define DS1307_H_

#include "../common/global.h"

#define DS1307_SECONDS_ADDR		0x00
#define DS1307_MINUTES_ADDR		0x01
#define DS1307_HOURS_ADDR		0x02
#define DS1307_DAY_ADDR			0x03
#define DS1307_DATE_ADDR		0x04
#define DS1307_MONTH_ADDR		0x05
#define DS1307_YEAR_ADDR		0x06
#define DS1307_CONTROL_ADDR		0x07

enum { kDS1307Mode12HR, kDS1307Mode24HR};
typedef u08 DS1307HourMode;

void ds1307_init(DS1307HourMode mode);
u08 ds1307_seconds(void);
u08 ds1307_minutes(void);
u08 ds1307_hours(void);
u08 ds1307_date(void);
void ds1307_set_seconds(u08 seconds);
void ds1307_set_minutes(u08 minutes);
void ds1307_set_hours(u08 hours);
void ds1307_set_year(u08 year);

#endif /* DS1307_H_ */
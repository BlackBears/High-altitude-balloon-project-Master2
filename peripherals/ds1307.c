/*
 * ds1307.c
 *
 * Created: 3/30/2012 11:07:13 AM
 *  Author: Owner
 */ 

#include "ds1307.h"
#include "../capabilities/i2c.h"
#include "../common/global.h"

#define SQWE	(1<<4);
#define RS0	(1<<0);
#define RS1	(1<<1);
#define DS1307_BASE_ADDRESS 0xD0
#define CH (1<<7)
#define HR (1<<6)

u08 device_data[2];

u08 ds1307_read_register(u08 reg);
void  ds1307_write_register(u08 reg,u08 data);

u08 hour_mode;
u08 ampm_mode;

#define HOUR_24 0
#define HOUR_12 1

static unsigned int uint2bcd(unsigned int ival)
{
	return ((ival / 10) << 4) | (ival % 10);
}
char dec2bcd(char num)
{
  return ((num/10 * 16) + (num % 10));
}
// Convert Binary Coded Decimal (BCD) to Decimal
char bcd2dec(char num)
{
  return ((num/16 * 10) + (num % 16));
}

void ds1307_init(DS1307HourMode mode)
{
	/*	To start the oscillator, we need to write CH = 0 (bit 7/reg 0) */
	u08 seconds = ds1307_read_register(DS1307_SECONDS_ADDR);
	seconds &= ~CH;
	ds1307_write_register(DS1307_SECONDS_ADDR,seconds);
	
	/*	set the mode */
	u08 hour = ds1307_read_register(DS1307_HOURS_ADDR);
	if( mode == kDS1307Mode12HR )
		hour &= ~HR;
	else
		hour |= HR;
	ds1307_write_register(DS1307_HOURS_ADDR, hour);
}

u08 ds1307_seconds(void)
{
	u08 seconds_h,seconds_l;
	u08 seconds = ds1307_read_register(DS1307_SECONDS_ADDR);
	/*	mask the CH bit */
	seconds &= ~CH;
	/*	get the rest of the high nibble */
	seconds_h = seconds >> 4;
	seconds_l = seconds & 0b00001111;
	return seconds_h * 10 + seconds_l;
}

u08 ds1307_minutes(void)
{
	u08 minutes_h,minutes_l;
	u08 minutes = ds1307_read_register(DS1307_MINUTES_ADDR);
	minutes_h = minutes >> 4;
	minutes_l = minutes & 0b00001111;
	return minutes_h * 10 + minutes_l;
}

u08 ds1307_hours(void)
{
	u08 hours = ds1307_read_register(DS1307_HOURS_ADDR);
	if( (hours & 0x40) == 0x40 ) {
		//	12 hour mode
		hour_mode = HOUR_12;
		ampm_mode=(hours & 0x20) >> 5;   // ampm_mode: 0-AM, 1-PM
		return bcd2dec(hours & 0x1F);
	}
	hour_mode = HOUR_24;
	ampm_mode = 0;
	return bcd2dec(hours & 0x3F);
}

u08 ds1307_date(void)
{
	u08 date_h,date_l;
	u08 date = ds1307_read_register(DS1307_DATE_ADDR);
	/*	mask the uppermost two bits */
	date &= ~(0b11000000);
	date_h = date >> 4;
	date_l = date & 0b00001111;
	return date_h * 10 + date_l;
}

void ds1307_set_seconds(u08 seconds)
{
	u08 bcd_seconds = uint2bcd(seconds);
	/* make sure CH bit is clear */
	bcd_seconds &= ~CH;
	ds1307_write_register(DS1307_SECONDS_ADDR,bcd_seconds);
}

void ds1307_set_minutes(u08 minutes)
{
	u08 bcd_minutes = uint2bcd(minutes);
	/*	make sure upper bit is clear */
	bcd_minutes &= ~(1<<7);
	ds1307_write_register(DS1307_MINUTES_ADDR,bcd_minutes);
}

void ds1307_set_hours(u08 hours)
{
	u08 hour_format = dec2bcd(hours);
	if( hour_mode ) {
		hour_format |= (1 << 6);
   	     if (ampm_mode)
	       hour_format |= (1 << 5);
         else
	       hour_format &= ~(1 << 5);
	}
	else {
		hour_format &= ~(1 << 6);
	}
	ds1307_write_register(DS1307_HOURS_ADDR,hour_format);
}

void ds1307_set_year(u08 year)
{
	 u08 bcd_year = uint2bcd(year);
	 ds1307_write_register(DS1307_YEAR_ADDR,bcd_year);
}

void  ds1307_write_register(u08 reg,u08 data)
{
	device_data[0] = reg;
	device_data[1] = data;
	i2cMasterSendNI(DS1307_BASE_ADDRESS,2,&device_data);
}

u08 ds1307_read_register(u08 reg)
{
	device_data[0] = reg;
	i2cMasterSendNI(DS1307_BASE_ADDRESS,1,&device_data);
	i2cMasterReceiveNI(DS1307_BASE_ADDRESS,1,&device_data);
	return device_data[0];
}

void ds1307_sqw_enable(BOOL state) {
	u08 ctl_reg = ds1307_read_register(DS1307_CONTROL_ADDR);
	if( state == TRUE )
		ctl_reg |= SQWE;
	else
		ctl_reg &= ~SQWE;
		
	ds1307_write_register(DS1307_CONTROL_ADDR,ctl_reg);
}

void ds1307_sqw_set_mode(ds1307_sqw_mode_t mode) {
	u08 ctl_reg = ds1307_read_register(DS1307_CONTROL_ADDR);
	if( mode == k_ds1307_sqw_mode_a ) {
		ctl_reg &= ~RS0;
		ctl_reg &= ~RS1;
	} 
	else if( mode == k_ds1307_sqw_mode_b )  {
		ctl_reg &= ~RS0;
		ctl_reg |= RS1;
	}
		ctl_reg &= ~0b00000010;
	else if( mode == k_ds1307_sqw_mode_c ) {
		ctl_reg |= RS1;
		ctl_reg &= ~RS0;
	}
	else if( mode == k_ds1307_sqw_mode_d )
		ctl_reg |= RS1 | RS0;
}
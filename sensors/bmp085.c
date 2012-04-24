/*
 * bmp085.c
 *
 * Created: 4/13/2012 11:14:22 PM
 *  Author: Administrator
 */ 

#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include "../capabilities/i2c.h"
#include "../common/global.h"
#include "../common/pindefs.h"
#include "bmp085.h"
#include "../capabilities/vfd.h"

#define BMP085_BASE_ADDRESS 0xEE

#define BMP085_AC1	0xAA
#define BMP085_AC2	0xAC
#define BMP085_AC3	0xAE
#define BMP085_AC4	0xB0
#define BMP085_AC5	0xB2
#define BMP085_AC6	0xB4
#define BMP085_B1	0xB6
#define BMP085_B2	0xB8
#define BMP085_MB	0xBA
#define BMP085_MC	0xBC
#define BMP085_MD	0xBE

#define BMP085_CTL	0xF4	//	control register
#define BMP085_RSLT 0xF6	//	conversion result register
#define BMP085_TEMP	0x2E	//	temperature
#define BMP085_P0	0x34	//	pressure with oversampling 0
#define BMP085_P1	0x74	//	pressure with oversampling 1
#define BMP085_P2	0xB4	//	pressure with oversampling 2
#define BMP085_P3	0xF4	//	pressure with oversampling 3

#define OSS 0	// Oversampling Setting (note: code is not set up to use other OSS values)

u08 device_data[2];
char buff[20];

/*	CALIBRATION VARIABLES */
short ac1;
short ac2; 
short ac3; 
unsigned short ac4;
unsigned short ac5;
unsigned short ac6;
short b1; 
short b2;
short mb;
short mc;
short md;

/*	FUNCTION PROTOTYPES */
void bmp085_read_calibration_data();
short bmp085_read_short(unsigned char address);
u08 bmp085_read_register(u08 reg);
int bmp085_read_word(u08 reg);
//u32 bmp085_read_temp();
long bmp085_read_temp();
//u32 bmp085_read_pressure();
long bmp085_read_pressure(void);

void bmp085_init() {
	//	configure EOC pin as input
	//DDR(BMP085_EOC_PORT) &= ~(1<<BMP085_EOC_PIN);
	DDR(BMP085_EOC_PORT) = 0b11111110;
	
	bmp085_read_calibration_data();
}

void bmp085_read_calibration_data() {
	ac1 = bmp085_read_short(BMP085_AC1);
	ac2 = bmp085_read_short(BMP085_AC2);
	ac3 = bmp085_read_short(BMP085_AC3);
	ac4 = bmp085_read_short(BMP085_AC4);
	ac5 = bmp085_read_short(BMP085_AC5);
	ac6 = bmp085_read_short(BMP085_AC6);
	b1 = bmp085_read_short(BMP085_B1);
	b2 = bmp085_read_short(BMP085_B2);
	mb = bmp085_read_short(BMP085_MB);
	mc = bmp085_read_short(BMP085_MC);
	md = bmp085_read_short(BMP085_MD);
	/*
	vfd_cls();
	sprintf(buff,"AC1=%02X B1=%02X MB=%02X",ac1,b1,mb);
	vfd_puts(buff);
	_delay_ms(5000);
	*/
}

void bmp085_convert(long* temperature, long* pressure) {
    long ut;
	long up;
	long x1, x2, b5, b6, x3, b3, p;
	unsigned long b4, b7;
	ut = bmp085_read_temp();
	ut = bmp085_read_temp();	// some bug here, have to read twice to get good data
	up = bmp085_read_pressure();
	up = bmp085_read_pressure();

	
	x1 = ((long)ut - ac6) * ac5 >> 15;
	x2 = ((long) mc << 11) / (x1 + md);
	b5 = x1 + x2;
	*temperature = (b5 + 8) >> 4;
	
	b6 = b5 - 4000;
	x1 = (b2 * (b6 * b6 >> 12)) >> 11;
	x2 = ac2 * b6 >> 11;
	x3 = x1 + x2;
	b3 = (((int32_t) ac1 * 4 + x3) + 2)/4;
	x1 = ac3 * b6 >> 13;
	x2 = (b1 * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (ac4 * (unsigned long) (x3 + 32768)) >> 15;
	b7 = ((unsigned long) up - b3) * (50000 >> OSS);
	p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	*pressure = p + ((x1 + x2 + 3791) >> 4);
}

u08 bmp085_read_register(u08 reg) {
	device_data[0] = reg;
	i2cMasterSendNI(BMP085_BASE_ADDRESS,1,&device_data);
	i2cMasterReceiveNI(BMP085_BASE_ADDRESS,1,&device_data);
	return device_data[0];
}

int bmp085_read_word(u08 reg) {
	device_data[0] = reg;
	i2cMasterSendNI(BMP085_BASE_ADDRESS,1,&device_data);
	_delay_ms(2);
	i2cMasterReceiveNI(BMP085_BASE_ADDRESS,2,&device_data);
	return (device_data[0] << 8) | device_data[1];
}

short bmp085_read_short(unsigned char address) {
    device_data[0] = address;
    i2cMasterSendNI(BMP085_BASE_ADDRESS,1,&device_data);
    i2cMasterReceiveNI(BMP085_BASE_ADDRESS,2,&device_data);
    
    short return_data = device_data[0] << 8;
    return_data |= device_data[1];
	return return_data;
}

long bmp085_read_temp() {
    //	begin temperature conversion
	device_data[0] = BMP085_CTL;
	device_data[1] = BMP085_TEMP;
	i2cMasterSendNI(BMP085_BASE_ADDRESS,2,&device_data);
	_delay_ms(10);
	return (long) bmp085_read_short(BMP085_RSLT);
}

long bmp085_read_pressure(void) {
    //	begin temperature conversion
	device_data[0] = BMP085_CTL;
	device_data[1] = BMP085_P0;
	i2cMasterSendNI(BMP085_BASE_ADDRESS,2,&device_data);
	
	_delay_ms(10);
	long p = 0;
	p = bmp085_read_short(BMP085_RSLT);
	p &= 0x0000FFFF;
	
	return p;
}
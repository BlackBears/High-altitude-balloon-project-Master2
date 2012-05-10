//////////////////////////////////////////////////////////////////////////////////////////
//	
//	File		: 'bmp085.h'
//	Author		: Alan K. Duncan <duncan.alan@mac.com>
//	Created		: 2012-05-01
//	Revised		: 2012-05-09
//	Version		: 1.0
//	Target MCU	: ATmega644A
//	
//	This file provides an interface to the BMP085 barometric pressure sensor via I2C 
//	interface.
//
//
//////////////////////////////////////////////////////////////////////////////////////////

#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include "../capabilities/i2c.h"
#include "../common/global.h"
#include "../common/pindefs.h"
#include "bmp085.h"
#include "../capabilities/uart2.h"

#define BMP085_BASE_ADDRESS 0xEE
#define BMP085_R 0xEF
#define BMP085_W 0xEE

/*	REGISTER ADDRESSES	*/
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

#define BMP085_I2C_INTERRUPT 0

BOOL device_error;
u08 device_data[2];

/*	CALIBRATION VARIABLES */
static short ac1 = 0;
static short ac2 = 0; 
static short ac3 = 0; 
static unsigned short ac4 = 0;
static unsigned short ac5 = 0;
static unsigned short ac6 = 0;
static short b1 = 0; 
static short b2 = 0;
static short mb = 0;
static short mc = 0;
static short md = 0;

/*	FUNCTION PROTOTYPES */
void BMP085_Calibration(void);
void bmp085Convert(long* temperature, long* pressure);
short bmp085ReadShort(unsigned char address);
long bmp085ReadTemp(void);
long bmp085ReadPressure(void);

#define BMP085_SHOW_CALIBRATION_VALUES 0

void bmp085_init(bmp085_t *device) {
	BMP085_Calibration();
#if BMP085_SHOW_CALIBRATION_VALUES
	char buffer[60];
	sprintf(buffer,"BMP085 calibration data: AC(1-6),%02d,%02d,%02d,%02d,%02d,%02d\r",ac1,ac2,ac3,ac4,ac5,ac6);
	uart1_puts(buffer);
	sprintf(buffer,"BMP085 calibration data: B(1-2), %02d,%02d\r",b1,b2);
	uart1_puts(buffer);
	sprintf(buffer,"BMP085 calibration data: M(b-d), %02d,%02d,%02d\r",mb,mc,md);
	uart1_puts(buffer);
#endif
}

void BMP085_Calibration(void)
{
	ac1 = bmp085ReadShort(0xAA);
	ac2 = bmp085ReadShort(0xAC);
	ac3 = bmp085ReadShort(0xAE);
	ac4 = bmp085ReadShort(0xB0);
	ac5 = bmp085ReadShort(0xB2);
	ac6 = bmp085ReadShort(0xB4);
	b1 = bmp085ReadShort(0xB6);
	b2 = bmp085ReadShort(0xB8);
	mb = bmp085ReadShort(0xBA);
	mc = bmp085ReadShort(0xBC);
	md = bmp085ReadShort(0xBE);
}

short bmp085ReadShort(unsigned char address)
{
	uint8_t data[2];
    data[0] = address;
#if BMP085_I2C_INTERRUPT == 0
    u08 i2c_status = i2cMasterSendNI(BMP085_BASE_ADDRESS,1,data); 
	
	/*
	if( i2c_status != I2C_OK) {
		sprintf(buffer,"*** ERROR while reading short | reg = %02X | status = %02X***",address,i2c_status);
		uart1_puts(buffer);
	}		
	*/
	_delay_ms(5);
	i2cMasterReceiveNI(BMP085_BASE_ADDRESS,2,data);
#else
	i2cMasterSend(BMP085_BASE_ADDRESS,1,data);
	_delay_ms(5);
	i2cMasterReceive(BMP085_BASE_ADDRESS,2,data);
#endif
    short return_data = (data[0] << 8);
    return_data |= data[1];
	//sprintf(buffer,"msb = %02X | msb = %02X\r",data[0],data[1]);
	//uart1_puts(buffer);
	return return_data;
}

long bmp085ReadTemp(void)
{
	//	begin temperature conversion
	uint8_t data[2];
	data[0] = BMP085_CTL;
	data[1] = BMP085_TEMP;
#if BMP085_I2C_INTERRUPT == 0
	i2cMasterSendNI(BMP085_BASE_ADDRESS,2,data);
#else
	i2cMasterSend(BMP085_BASE_ADDRESS,2,data);
#endif
	_delay_ms(10);
	return (long) bmp085ReadShort(BMP085_RSLT);
}

long bmp085ReadPressure(void)
{
	//	begin temperature conversion
	uint8_t data[2];
	data[0] = BMP085_CTL;
	data[1] = BMP085_P0;
#if BMP085_I2C_INTERRUPT == 0
	u08 i2c_status = i2cMasterSendNI(BMP085_BASE_ADDRESS,2,data);
	if( i2c_status != I2C_OK ) {
		uartSendString_P(1,"*** ERROR: while reading barometric pressure ***");
	}
#else
	i2cMasterSend(BMP085_BASE_ADDRESS,2,data);
#endif
	_delay_ms(10);
	long p = 0;
	p = bmp085ReadShort(BMP085_RSLT);
	p &= 0x0000FFFF;
	return p;
}

void bmp085Convert(long* temperature, long* pressure)
{
	long ut;
	long up;
	long x1, x2, b5, b6, x3, b3, p;
	unsigned long b4, b7;
	
	ut = bmp085ReadTemp();
	up = bmp085ReadPressure();
	
	//sprintf(buffer,"up = %ld ut = %ld",up,ut); uart1_puts(buffer);
	
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
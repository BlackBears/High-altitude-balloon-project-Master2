/*
 * gps2.c
 *
 * Created: 4/13/2012 11:33:15 PM
 *  Author: Administrator
 */ 

/*  OPCODES FOR OUR I2C INTERFACE */

#include "gps2.h"

#define E_W_DIR     0x20    // East-West direction (1 char "E" or "W"
#define E_W_VEL     0x21    // East_West velocity (2 bytes u16 in m/s * 100 )
#define N_S_DIR     0x22    // North-South direction (1 char "N" or "S")
#define N_S_VEL     0x23    // North-South velocity (2 bytes u16 in m/s * 100)
#define VERT_DIR    0x24    // vertical velocity (1 char "U" or "D")
#define VERT_VEL    0x25    // vertical velocity (2 bytes u16 in m/s *10)
#define LAT			0x40	// return 4 bytes representing the latitude
#define LON			0x41	// return 4 bytes representing the longitude
#define FIX_TIME	0x50	// return the time of the most recent fix, returns 6 bytes

#define DEBUG_ON	0x60	// turn on debugging mode, returns I2C_DEBUG_CONFIRM_BYTE
#define DEBUG_OFF	0x61	// turn off debugging mode, returns I2C_DEBUG_CONFIRM_BYTE
#define DX_MODE_ON	0x70	// turn on diagnostic mode (uses fake GPS data), returns I2C_DEBUG_CONFIRM_BYTE
#define DX_MODE_OFF	0x71	// turn off diagnostic mode, returns I2C_DEBUG_CONFIRM_BYTE
#define TEST		0x02	// flash the test LED, returns I2C_DEBUG_CONFIRM_BYTE

#define I2C_DEBUG_CONFIRM_BYTE	0xF0

#define GPS2_I2C_SLAVE_ADDRESS 0xA0

BOOL gps2_set_debug_mode_on(u08 status) {
	u08 data[1];
	data[0] = (status)?DEBUG_ON:DEBUG_OFF;
	i2cMasterSendNI(GPS2_I2C_SLAVE_ADDRESS,1,&data);
	i2cMasterReceiveNI(GPS2_I2C_SLAVE_ADDRESS,1,&data);
	if( data == I2C_DEBUG_CONFIRM_BYTE ) return TRUE;
	return FALSE;
}

BOOL gps2_set_diagnostic_mode_on(u08 status) {
	u08 data[1];
	data[0] = (status)?DX_MODE_ON:DX_MODE_OFF
	i2cMasterSendNI(GPS2_I2C_SLAVE_ADDRESS,1,&data);
	i2cMasterReceiveNI(GPS2_I2C_SLAVE_ADDRESS,1,&data);
	if( data == I2C_DEBUG_CONFIRM_BYTE ) return TRUE;
	return FALSE;
}
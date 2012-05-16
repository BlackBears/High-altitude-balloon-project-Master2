
#include "cellular.h"
#include "../capabilities/i2c.h"

#define CELLULAR_BASE_ADDRESS 0xBB

#define CELLULAR_HOUR_REG 0x00
#define CELLULAR_MINUTE_REG	0x01
#define CELLULAR_SECOND_REG	0x02
#define CELLULAR_ALT_REG	0x03
#define CELLULAR_TEMP_REG	0x07
#define CELLULAR_BUS_V_REG	0x09	//	bus voltage register

static void cellular_write_register(u08 reg,u08 data)
{
	u08 device_data[2];
	device_data[0] = reg;
	device_data[1] = data;
	cli();
	i2cMasterSendNI(CELLULAR_BASE_ADDRESS,2,device_data);
	sei();
}

static int8_t cellular_read_register(u08 reg)
{
	u08 device_data[2];
	device_data[0] = reg;
	i2cMasterSendNI(CELLULAR_BASE_ADDRESS,1,device_data);
	_delay_ms(10);
	i2cMasterReceiveNI(CELLULAR_BASE_ADDRESS,1,device_data);
	return device_data[0];
}

void cellular_set_time(time_t t) {
	cellular_write_register(CELLULAR_HOUR_REG, t.hour);
	cellular_write_register(CELLULAR_MINUTE_REG, t.minute);
	cellular_write_register(CELLULAR_SECOND_REG, t.second);
}

void cellular_set_altitude(int32_t a) {
	cellular_write_register(CELLULAR_ALT_REG, a>>24);
	cellular_write_register(CELLULAR_ALT_REG+1, (uint8_t)(a>>16));
	cellular_write_register(CELLULAR_ALT_REG+2, (uint8_t)(a>>8));
	cellular_write_register(CELLULAR_ALT_REG+3, (uint8_t)a & ~0xFFFFFF00);
}

void cellular_set_capsule_temp(int16_t t) {
	cellular_write_register(CELLULAR_TEMP_REG, t>>8);
	cellular_write_register(CELLULAR_TEMP_REG+1, (uint8_t)(t & ~0xFF00));
}

void cellular_set_bus_voltage(uint16_t v) {
	cellular_write_register(CELLULAR_BUS_V_REG, v>>8);
	cellular_write_register(CELLULAR_BUS_V_REG+1, (uint8_t)(v & ~0xFF00));
}
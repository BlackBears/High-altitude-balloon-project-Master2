
#include "control_panel.h"

void control_panel_init() {
	// make our dx pins output for LEDs
	DDR(GPS1_DX_PORT) |= (1<<GPS_1_DX_PIN);
	DDR(GPS2_DX_PORT) |= (1<<GPS_2_DX_PIN);
	DDR(LOGGER_DX_PORT) |= (1<<LOGGER_DX_PIN);
	DDR(TEMP_DX_PORT) |= (1<<TEMP_DX_PIN);
	DDR(OTHER_DX_PORT) |= (1<<OTHER_DX_PIN);
	
	DDR(FLIGHT_MODE_PORT) &= ~(1<<FLIGHT_MODE_PIN);
	EICRB |= (1<<ISC50);
	EICRB &= ~(1<<ISC51);
	
	//	set our initial status based on our FLIGHT_MODE_PIN
	//	inflight is HIGH, preflight is LOW
	if( PIN(FLIGHT_MODE_PORT) & (1<<FLIGHT_MODE_PIN) )
		control_panel_status = k_control_panel_status_inflight;
	else
		control_panel_status = k_control_panel_status_preflight;
}

void control_panel_indicator_set_state(control_panel_indicator_t index, dx_indicator_state_t state) {
	indicators[index].state  = state;
/*
	if( indicator == k_dx_indicator_gps1 ) {
		if( state == TRUE )
			GPS1_DX_PORT |= (1<<GPS_1_DX_PIN);
		else
			GPS1_DX_PORT &= ~(1<<GPS_1_DX_PIN);
	}
	if( indicator == k_dx_indicator_gps2 ) {
		if( state == TRUE )
			GPS2_DX_PORT |= (1<<GPS_2_DX_PIN);
		else GPS2_DX_PORT &= ~(1<<GPS_2_DX_PIN);
	}
	if( indicator == k_dx_indicator_logger ) {
		if( state == TRUE )
			LOGGER_DX_PORT |= (1<<LOGGER_DX_PIN);
		else
			LOGGER_DX_PORT &= ~(1<<LOGGER_DX_PIN);
	}
	if( indicator == k_dx_indicator_temp ) {
		if( state == TRUE )
			TEMP_DX_PORT |= (1<<TEMP_DX_PIN);
		else
			TEMP_DX_PORT &= ~(1<<TEMP_DX_PIN);
	}
	if( indicator == k_dx_indicator_other ) {
		if( state == TRUE )
			OTHER_DX_PORT |= (1<<OTHER_DX_PIN);
		else
			OTHER_DX_PORT &= ~(1<<OTHER_DX_PIN);
	}
	*/
}

void control_panel_indicator_update(void) {
	for( u08 i = 0; i < 6; i++ ) {
		
	}	/*	iterate indicators */
}

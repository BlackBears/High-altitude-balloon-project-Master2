
#include <avr/io.h>
#include "../common/global.h"

#define DX_INDICATOR_PORT	PORTE
#define GPS1_DX_PORT		DX_INDICATOR_PORT
#define GPS2_DX_PORT		DX_INDICATOR_PORT
#define LOGGER_DX_PORT		DX_INDICATOR_PORT
#define TEMP_DX_PORT		DX_INDICATOR_PORT
#define OTHER_DX_PORT		DX_INDICATOR_PORT
#define FLIGHT_MODE_PORT	PORTE	//	DON'T CHAGE

#define GPS_1_DX_PIN		PE0
#define GPS_2_DX_PIN		PE1
#define LOGGER_DX_PIN		PE2
#define TEMP_DX_PIN			PE3
#define OTHER_DX_PIN		PE4
#define FLIGHT_MODE_PIN		PE5	//	DON'T CHANGE

enum {
	k_control_panel_status_preflight,
	k_control_panel_status_inflight
};
typedef u08 control_panel_status_t;

enum {
	k_dx_indicator_gps1,
	k_dx_indicator_gps2,
	k_dx_indicator_logger,
	k_dx_indicator_temp,
	k_dx_indicator_other
};
typedef u08 control_panel_indicator_t;

enum {
	k_dx_indicator_error,
	k_dx_indicator_good,
};
typedef u08 dx_indicator_state_t;

typedef struct {
	dx_indicator_state_t state;
	BOOL is_on;
} dx_indicator_t;

dx_indicator_t indicators[6];

control_panel_status_t control_panel_status;

void control_panel_init();
void control_panel_indicator_set_state(control_panel_indicator_t indicator, dx_indicator_state state);
void control_panel_indicator_update(void);
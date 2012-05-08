/*
 * altimeter.h
 *
 * Created: 5/5/2012 12:12:00 AM
 *  Author: Administrator
 */ 


#ifndef ALTIMETER_H_
#define ALTIMETER_H_

#include "../common/global.h"

typedef enum {
	ALTIMETER_DESCENDING = -1,
	ALTIMETER_STATIC = 0,
	ALTIMETER_ASCENDING = 1
} altimeter_direction_t;
typedef u08 altimeter_source_t;

struct altimeter_value_t {
	u32 altitude;
	altimeter_source_t source;
	struct altimeter_value_t *next;
};

struct altimeter_list {
	struct altimeter_value_t *head;
	struct altimeter_value_t *tail;
	u08 size;
	u08 ascending_count;
	u08 descending_count;
};	
typedef struct altimeter_list altimeter_list_t;

altimeter_list_t *a_list;		//	our linked list of altimeter_value_t strtucs

void altimeter_init(void);
void altimeter_add_element(s32 value, altimeter_source_t source);	//	value is in meters if GPS, Pascals if BMP085
altimeter_direction_t altimeter_direction();

	
#endif /* ALTIMETER_H_ */
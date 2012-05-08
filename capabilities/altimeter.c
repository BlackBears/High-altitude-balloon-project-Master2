/*
 * altimeter.c
 *
 * Created: 5/5/2012 12:12:46 AM
 *  Author: Administrator
 */ 

#include "altimeter.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define ALTITUDE_BUFFER_SIZE 30;

static const altimeter_source_t ALTIMETER_SOURCE_GPS = 0;
static const altimeter_source_t ALTIMETER_SOURCE_BMP085 = 1;

void _altitude_remove_element(void);

void altimeter_add_element(s32 value, altimeter_source_t source)
{
	//	first pop an item off the stack depending on our size
	if( a_list->size >= 30 )
		_altitude_remove_element();
	else
		a_list->size++;
	struct altimeter_value_t* p = malloc( 1 * sizeof(*p) );
	
	s32 last_altitude = a_list->tail->altitude;
	if( value > last_altitude  ) {
		a_list->ascending_count++;
		a_list->descending_count--;
	} 
	else {
		a_list->ascending_count--;
		a_list->descending_count++;
	}

	if( source == ALTIMETER_SOURCE_GPS )
		p->altitude = value;
	else {
		//	this is a barometric pressure value
		//	so must convert to meters
		/*temp = (double) pressure/101325;
		temp = 1-pow(temp, 0.19029);
		altitude = round(44330*temp);
		printf("Altitude: %ldm\n\n", altitude);*/
		double temp = (double)value/101325;
		temp = 1-pow(temp,0.19029);
		p->altitude = round(44330 * temp);
	}
	p->next = NULL;
	if( a_list->head == NULL && a_list->tail == NULL) { a_list->head = a_list->tail = p; return; }
	if( a_list->head == NULL || a_list->tail == NULL) { free(a_list); return; }
	
	a_list->tail->next = p;
    a_list->tail = p;
}

/* This is a queue and it is FIFO, so we will always remove the first element */
void _altitude_remove_element(void)
{
	struct altimeter_value_t *h = NULL;
	struct altimeter_value_t *p = NULL;
	if( a_list == NULL ) return;
	if( a_list->head == NULL || a_list->tail == NULL) return;

	h = a_list->head;
	p = h->next;
	free(h);
	a_list->head = p;
	if( NULL == a_list->head )  a_list->tail = a_list->head;   /* The element tail was pointing to is free(), so we need an update */
}

altimeter_direction_t altimeter_direction() {
	if( a_list->ascending_count >= 20 && a_list->descending_count <= 10) {
		return ALTIMETER_ASCENDING;
	}
	if( a_list->descending_count >= 20 && a_list->ascending_count <= 10) {
		return ALTIMETER_DESCENDING;
	}
	return ALTIMETER_STATIC;
}

/*	HELPER FUNCTIONS */
void list_free(void) {
	while( a_list->head ) {
		_altitude_remove_element();
    }
}
void list_new(void)
{
  a_list = malloc( 1 * sizeof(*a_list));

  a_list->head = a_list->tail = NULL;
}
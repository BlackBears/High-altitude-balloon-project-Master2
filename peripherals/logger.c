
#include "../capabilities/fat/diskio.h"
#include <avr/io.h>

FATFS FileSystemObject;
02
 
03
if(f_mount(0, &FileSystemObject)!=FR_OK) {
04
  //flag error
05
 }


/* we need to checkin with the SD card every 10 ms to see if something has gone awry. 
We can use a 10 ms pulse on INT7 (PE7) to generate this pulse.  One accurate way of doing
this is to use a small ATTiny12 or the like to generate this pulse. */

BOOL logger_init(void) {
	//	enable external interrupt on PE7
	EICRB |= (1<<ISC70) | (1<<ISC71);
	EIMSK |= (1<<INT7);
	sei();
	
	//	continue other initialization here.
	if( f_mount(0, &FileSystemObject) != FR_OK ) {
		return FALSE;
	}
	return TRUE;
}
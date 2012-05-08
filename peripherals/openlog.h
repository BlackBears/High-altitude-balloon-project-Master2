/*
 * openlog.h
 *
 * Created: 4/24/2012 9:02:00 AM
 *  Author: Administrator
 */ 


#ifndef OPENLOG_H_
#define OPENLOG_H_

#include "../common/global.h"
#include <stddef.h>


#define OPEN_LOG_UART	UART1

void open_log_init(void);
void open_log_set_pwr(BOOL pwr_state);
void open_log_reset();
void open_log_reset_nack();
void open_log_command_mode();
void open_log_write_test(void);
void open_log_ls(char *buffer, size_t size);


#endif /* OPENLOG_H_ */
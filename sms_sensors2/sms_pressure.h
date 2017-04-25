#ifndef SMS_PRESSURE_H_
#define SMS_PRESSURE_H_

#include "ms58.h"

#define SMS_PRESSURE_PRESS_LEN		4
#define SMS_PRESSURE_TEMP_LEN		4
#define SMS_PRESSURE_CHAR_LEN		(SMS_PRESSURE_PRESS_LEN + SMS_PRESSURE_TEMP_LEN)


void pressure_startup(void);
void pressure_read_data(void);
void pressure_calculate(void);


#endif

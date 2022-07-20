#ifndef __VOLTAGES_H
#define __VOLTAGES_H
 #include "stm32f1xx_hal.h"
struct Voltages_exp
{
	float *arr;
	uint16_t counter;
} ;
extern struct Voltages_exp Voltage_Testing;
#endif /* __VOLTAGES_H*/

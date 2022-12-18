#ifndef __NODE_MOTOR_H__
#define __NODE_MOTOR_H__
#include "stdio.h"
#include "stdlib.h"
#include <string.h>
#include <stdint.h>



typedef struct
{
	uint32_t left_cnt;
	uint32_t right_cnt;
}SPEED_PULSE_CNT;

void node_tx_task(void const * argument);



#endif

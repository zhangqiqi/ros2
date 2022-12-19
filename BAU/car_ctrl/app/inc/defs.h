#ifndef __DEFS__
#define __DEFS__

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "stdlib.h"
#include <string.h>
#include "cli_uart.h"
#include "node_motor.h"


#include "odometry.h"
#include "PID.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart2;

extern osSemaphoreId init_complete;
extern osSemaphoreId sem_odometry_cal;
extern osSemaphoreId semlog_tx_cpt;
extern osSemaphoreId semlog_rx_cpt;
extern osSemaphoreId semnode_tx_cpt;
extern osSemaphoreId semnode_rx_cpt;

extern osMutexId mutex_log;

extern QueueHandle_t queue_odom_handle;

#endif /*__DEFS__*/



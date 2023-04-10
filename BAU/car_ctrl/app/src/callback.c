#include "defs.h"
extern uint32_t Milemeter_L_Motor,Milemeter_R_Motor;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint32_t cnt = 0;
	if (htim == &htim6)
	{
		cnt++;
		if(cnt >= 5)
		{
			cnt = 0;
			osSemaphoreRelease(sem_odometry_cal);
#if 0			
			LOGI("Milemeter_L_Motor %d Milemeter_R_Motor %d", Milemeter_L_Motor, Milemeter_R_Motor);
			Milemeter_L_Motor = 0;
			Milemeter_R_Motor = 0;
#endif			
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_0)
	{
		Milemeter_R_Motor++;
//		LOGI("Milemeter_L_Motor %d", Milemeter_L_Motor);
	}
	else if (GPIO_Pin == GPIO_PIN_1)
	{
		Milemeter_R_Motor++;
	}
	else if (GPIO_Pin == GPIO_PIN_2)
	{
		Milemeter_L_Motor++;
//		LOGI("Milemeter_R_Motor %d", Milemeter_R_Motor);
	}
	else if (GPIO_Pin == GPIO_PIN_3)
	{
		Milemeter_L_Motor++;
	}
	
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		osSemaphoreRelease(semlog_tx_cpt);
	}
	else if (huart == &huart2)
	{
		osSemaphoreRelease(semnode_tx_cpt);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		osSemaphoreRelease(semlog_rx_cpt);		
	}
	else if (huart == &huart2)
	{
		osSemaphoreRelease(semnode_rx_cpt);
	}
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		osSemaphoreRelease(semlog_rx_cpt);		
	}
	else if (huart == &huart2)
	{
		osSemaphoreRelease(semnode_rx_cpt);
	}

}



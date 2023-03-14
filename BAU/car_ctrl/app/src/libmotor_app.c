#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include <stdio.h>
#include <stdarg.h>

#include "motor.h"

#ifdef BAU_SIMULATION

#else

#include "defs.h"
#endif

struct MOTOR *left_motor;
struct MOTOR *right_motor;

static struct MOTOR_MANAGER *motor_manager;


static uint32_t get_motor_encoder_cnt(TIM_HandleTypeDef *htim)
{
	uint32_t encoder_cnt = 0;
	int32_t dir = 0;
	
	dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(htim);
	if ((1 == dir) && (__HAL_TIM_GET_COUNTER(htim) != 0))
	{
		encoder_cnt = htim->Init.Period - __HAL_TIM_GET_COUNTER(htim);
	}
	else
	{
		encoder_cnt = __HAL_TIM_GET_COUNTER(htim);
	}
	__HAL_TIM_SET_COUNTER(htim, 0);
	
	return encoder_cnt;
}


int32_t bau_motor_left_ctrl_out(void *ctrl_out_handle, float target)
{
#ifdef BAU_SIMULATION
	float *encoder_cnt = (float *)ctrl_out_handle;
	*encoder_cnt += target * 0.777;
#else
	TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)ctrl_out_handle;

	if (target > 0)
	{
		int32_t out = htim->Instance->CCR2 + target;
		htim->Instance->CCR1 = 0;
		htim->Instance->CCR2 = out > 0 ? out : 0;
	}
	else if (target < 0)
	{
		int32_t out = htim->Instance->CCR1 + target;
		htim->Instance->CCR1 = out > 0 ? out : 0;
		htim->Instance->CCR2 = 0;
	}
	else
	{
		htim->Instance->CCR1 = 0;
		htim->Instance->CCR2 = 0;

	}
#endif

	return 0;
}

int32_t bau_motor_right_ctrl_out(void *ctrl_out_handle, float target)
{
#ifdef BAU_SIMULATION
	float *encoder_cnt = (float *)ctrl_out_handle;
	*encoder_cnt += target * 0.777;
#else
	TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)ctrl_out_handle;

	if (target > 0)
	{
		int32_t out = htim->Instance->CCR4 + target;
		htim->Instance->CCR3 = 0;
		htim->Instance->CCR4 = out > 0 ? out : 0;	

	}
	else if (target < 0)
	{
		int32_t out = htim->Instance->CCR3 + target;
		htim->Instance->CCR3 = out > 0 ? out : 0;
		htim->Instance->CCR4 = 0;
	}
	else
	{
		htim->Instance->CCR3 = 0;
		htim->Instance->CCR4 = 0;	
	}
#endif

	return 0;
}


float bau_motor_encoder_read(void *handle)
{
#ifdef BAU_SIMULATION
	float *encoder_cnt = (float *)handle;
#else
	float encoder_cnt = get_motor_encoder_cnt(handle);
#endif

	return encoder_cnt;
}


void libmotor_task_thread_exec(void const *argument)
{
#ifdef BAU_SIMULATION
	float left_encoder_cnt = 0;
	motor_set_ctrl_out_if(left_motor, &left_encoder_cnt, bau_motor_ctrl_out);
	motor_set_counter_read_if(left_motor, &left_encoder_cnt, bau_motor_encoder_read);

	float right_encoder_cnt = 0;
	motor_set_ctrl_out_if(right_motor, &right_encoder_cnt, bau_motor_ctrl_out);
	motor_set_counter_read_if(right_motor, &right_encoder_cnt, bau_motor_encoder_read);
#else
	motor_set_ctrl_out_if(left_motor, &htim4, bau_motor_left_ctrl_out);
	motor_set_counter_read_if(left_motor, &htim2, bau_motor_encoder_read);

	motor_set_ctrl_out_if(right_motor, &htim4, bau_motor_right_ctrl_out);
	motor_set_counter_read_if(right_motor, &htim3, bau_motor_encoder_read);
#endif

	do
	{
		osDelay(10);
		motor_exec(motor_manager, 10 * 1000);
	} while (true);
}


void libmotor_app_init(void)
{
	motor_manager = motor_init();

	left_motor = motor_create(motor_manager, 10 * 1000, NULL);
	right_motor = motor_create(motor_manager, 10 * 1000, NULL);

	osThreadDef(libmotor_task_thread, libmotor_task_thread_exec, osPriorityNormal, 0, 1024);
	osThreadCreate(osThread(libmotor_task_thread), NULL);
}

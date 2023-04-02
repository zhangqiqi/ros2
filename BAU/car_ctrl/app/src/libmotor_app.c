#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include <stdio.h>
#include <stdarg.h>

#include "motor.h"
#include "counter.h"

#ifdef BAU_SIMULATION

#else

#include "defs.h"
#endif

struct MOTOR *left_motor;
struct MOTOR *right_motor;

struct MOTOR *left_position_ctrl;
struct MOTOR *right_position_ctrl;
static struct MOTOR_MANAGER *motor_manager;


int32_t bau_motor_left_ctrl_out(struct MOTOR *motor, void *ctrl_out_handle, float target)
{
#ifdef BAU_SIMULATION
	float *encoder_cnt = (float *)ctrl_out_handle;
	*encoder_cnt += target * 0.777;
#else
	TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)ctrl_out_handle;
	int32_t abs_target = motor_get_target(motor);

	if (abs_target > 0)
	{
		htim->Instance->CCR1 = 0;
		htim->Instance->CCR2 = htim->Instance->CCR2 + target;
	}
	else if (abs_target < 0)
	{
		htim->Instance->CCR1 = htim->Instance->CCR1 - target;
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

int32_t bau_motor_right_ctrl_out(struct MOTOR *motor, void *ctrl_out_handle, float target)
{
#ifdef BAU_SIMULATION
	float *encoder_cnt = (float *)ctrl_out_handle;
	*encoder_cnt += target * 0.777;
#else
	TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)ctrl_out_handle;
	int32_t abs_target = motor_get_target(motor);

	if (abs_target > 0)
	{
		htim->Instance->CCR3 = htim->Instance->CCR3 + target;
		htim->Instance->CCR4 = 0;
	}
	else if (abs_target < 0)
	{
		htim->Instance->CCR3 = 0;
		htim->Instance->CCR4 = htim->Instance->CCR4 - target;
	}
	else
	{
		htim->Instance->CCR3 = 0;
		htim->Instance->CCR4 = 0;	
	}
#endif

	return 0;
}


int32_t bau_motor_encoder_read(struct COUNTER *counter, void *handle)
{
#ifdef BAU_SIMULATION
	float *encoder_cnt = (float *)handle;
#else
	TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)handle;

	int32_t dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(htim);
	int32_t cnt = __HAL_TIM_GET_COUNTER(htim);

	counter_add_abs_value(counter, 1 == dir ? CUD_BACKWARD : CUD_FORWARD, cnt);
#endif

	return 0;
}


int32_t bau_motor_position_ctrl_out(struct MOTOR *motor, void *ctrl_out_handle, float target)
{
	struct MOTOR *dst_speed_motor = (struct MOTOR *)ctrl_out_handle;

	motor_set_target(dst_speed_motor, target);

	return 0;
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
	struct COUNTER *left_counter = counter_create(2, 32767, -32768);
	motor_set_counter(left_motor, left_counter);
	motor_set_ctrl_out_if(left_motor, &htim4, bau_motor_left_ctrl_out);
	motor_set_counter_update_if(left_motor, &htim2, bau_motor_encoder_read);

	struct COUNTER *right_counter = counter_create(2, 32767, -32768);
	motor_set_counter(right_motor, right_counter);
	motor_set_ctrl_out_if(right_motor, &htim4, bau_motor_right_ctrl_out);
	motor_set_counter_update_if(right_motor, &htim3, bau_motor_encoder_read);

	// struct COUNTER *left_position_counter = counter_create(2, 32767, -32768);
	// motor_set_counter(left_position_ctrl, left_position_counter);
	// motor_set_ctrl_out_if(left_position_ctrl, left_motor, bau_motor_position_ctrl_out);
	// motor_set_counter_update_if(left_position_ctrl, &htim2, bau_motor_encoder_read);

	// struct COUNTER *right_position_counter = counter_create(2, 32767, -32768);
	// motor_set_counter(right_position_ctrl, right_position_counter);
	// motor_set_ctrl_out_if(right_position_ctrl, right_motor, bau_motor_position_ctrl_out);
	// motor_set_counter_update_if(right_position_ctrl, &htim3, bau_motor_encoder_read);

	// motor_state_clear(left_position_ctrl);
	// motor_state_clear(right_position_ctrl);

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

	PIDController position_ring_params = {
		.Kp = 0.9,
		.Ki = 0,
		.Kd = 0,
		.tau = 0.02,
		.limMin = -30.0,
		.limMax = 30.0,
		.limMinInt = -5.0,
		.limMaxInt = 5.0,
	};

	PIDController speed_ring_params = {
		.Kp = 0.9,
		.Ki = 0,
		.Kd = 0,
		.tau = 0.02,
		.limMin = -15.0,
		.limMax = 15.0,
		.limMinInt = -5.0,
		.limMaxInt = 5.0,
	};

	// left_position_ctrl = motor_create(motor_manager, 10 * 1000, &position_ring_params, MCT_POSITION_RING_CTRL);
	// right_position_ctrl = motor_create(motor_manager, 10 * 1000, &position_ring_params, MCT_POSITION_RING_CTRL);

	left_motor = motor_create(motor_manager, 10 * 1000, &speed_ring_params, MCT_SPEED_RING_CTRL);
	right_motor = motor_create(motor_manager, 10 * 1000, &speed_ring_params, MCT_SPEED_RING_CTRL);

	osThreadDef(libmotor_task_thread, libmotor_task_thread_exec, osPriorityNormal, 0, 1024);
	osThreadCreate(osThread(libmotor_task_thread), NULL);
}

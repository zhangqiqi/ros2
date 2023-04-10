#include "motor.h"
#include "counter.h"

#include <stdio.h>
#include <math.h>


/**
 * @brief 速度环输出控制
 */
int32_t motor_speed_ring_ctrl_out(struct MOTOR *motor, void *ctrl_out_handle, float target)
{
	struct COUNTER *counter = (struct COUNTER *)ctrl_out_handle;

	int32_t cur = 0;
	counter_get_rel_value(counter, 2, &cur);

	cur += target * 0.777;

	if (cur >= 0)
	{
		return counter_add_rel_value(counter, CUD_FORWARD, cur);
	}
	else
	{
		return counter_add_rel_value(counter, CUD_BACKWARD, cur);
	}

	return 0;
}


/**
 * @brief 速度环采样
 */
int32_t motor_speed_ring_read_counter(struct COUNTER *counter, void *handle)
{
	return 0;
}


/**
 * @brief 位置环输出控制
 */
int32_t motor_position_ring_ctrl_out(struct MOTOR *motor, void *ctrl_out_handle, float target)
{
	struct MOTOR *speed_ring = (struct MOTOR *)ctrl_out_handle;

	motor_set_target(speed_ring, target);

	return 0;
}


/**
 * @brief 位置环采样
 */
int32_t motor_position_ring_read_counter(struct COUNTER *counter, void *handle)
{
	return 0;
}


int main(int argc, char **argv)
{
	struct MOTOR_MANAGER *motor_manager = motor_init();

	struct COUNTER *counter = counter_create(10, 32767, -32768);

	PIDController speed_ring_params = {
		.Kp = 1.5,
		.Ki = 0,
		.Kd = 0,
		.tau = 0.02,
		.limMin = -50.0,
		.limMax = 50.0,
		.limMinInt = -5.0,
		.limMaxInt = 5.0,
	};

	PIDController position_ring_params = {
		.Kp = 0.9,
		.Ki = 0,
		.Kd = 0,
		.tau = 0.02,
		.limMin = -50.0,
		.limMax = 50.0,
		.limMinInt = -5.0,
		.limMaxInt = 5.0,
	};
	struct MOTOR *position_ring = motor_create(motor_manager, 10 * 1000, &position_ring_params, MCT_POSITION_RING_CTRL);
	struct MOTOR *speed_ring = motor_create(motor_manager, 10 * 1000, &speed_ring_params, MCT_SPEED_RING_CTRL);

	motor_set_counter(speed_ring, counter);
	motor_set_ctrl_out_if(speed_ring, counter, motor_speed_ring_ctrl_out);
	motor_set_counter_update_if(speed_ring, NULL, motor_speed_ring_read_counter);

	motor_set_counter(position_ring, counter);
	motor_set_ctrl_out_if(position_ring, speed_ring, motor_position_ring_ctrl_out);
	motor_set_counter_update_if(position_ring, NULL, motor_position_ring_read_counter);

	float target = 5000;

	motor_set_target(position_ring, target);

	while (fabs(motor_get_target_ratio(position_ring) - 1.0) > 0.001 || fabs(motor_get_target_ratio(speed_ring) - 1.0) > 0.001)
	{
		// sleep(1);
		printf("\r\n");
		motor_exec(motor_manager, 10 * 1000);
	}

	return 0;
}

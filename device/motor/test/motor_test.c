#include "motor.h"

#include <stdio.h>




int32_t test_motor_ctrl_out(void *ctrl_out_handle, float target)
{
	float *encoder_cnt = (float *)ctrl_out_handle;
	*encoder_cnt += target * 0.777;

	return 0;
}


float test_motor_read_encoder(void *handle)
{
	float *encoder_cnt = (float *)handle;

	return *encoder_cnt;
}

int main(int argc, char **argv)
{
	struct MOTOR_MANAGER *motor_manager = motor_init();
	struct MOTOR *test_motor = motor_create(motor_manager, 10 * 1000, NULL);

	float encoder_cnt = 0;
	motor_set_ctrl_out_if(test_motor, &encoder_cnt, test_motor_ctrl_out);
	motor_set_encoder_read_if(test_motor, &encoder_cnt, test_motor_read_encoder);

	motor_set_target(test_motor, 1000);

	while (fabs(1000 - encoder_cnt) > 0.001)
	{
		motor_exec(motor_manager, 10 * 1000);
	}

	return 0;
}

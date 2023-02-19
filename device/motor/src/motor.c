#include "motor.h"


/**
 * @brief 电机描述结构体
 */
struct MOTOR {
	struct MOTOR_ENCODER *encoder;      /**< 编码器对象 */

};


/**
 * @brief 设置电机的编码器值读取接口
 * @param motor 目标电机对象
 * @param encoder_handle 编码器读取句柄
 * @param read_if 编码器读取接口
 * @return int32_t 0 设置成功 其它 设置失败
 */
int32_t motor_set_encoder_read_if(struct MOTOR *motor, void *encoder_handle, MOTOR_READ_ENCODER read_if)
{
	int32_t ret = 0;

	if (NULL == motor)
	{
		return -1;
	}

	motor->encoder_handle = encoder_handle;
	motor->encoder_read = read_if;

	return ret;
}


/**
 * @brief 执行目标电机的控制
 * @param motor 目标电机对象
 */
void motor_exec(struct MOTOR *motor)
{

}

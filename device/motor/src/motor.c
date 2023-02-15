#include "motor.h"


/**
 * @brief 电机描述结构体
 */
struct MOTOR {

	int32_t encoder_cnt[MOROT_ENCODER_CNT_NUM];      /**< 电机编码器计数数组 */
	void *encoder_handle;      /**< 编码器读取对象指针 */
	MOTOR_READ_ENCODER encoder_read;      /**< 编码器值读取接口 */

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
 * @brief 更新电机的编码器值
 * @param motor 目标电机对象
 */
static void motor_encoder_cnt_update(struct MOTOR *motor)
{
	if (NULL == motor->encoder_read)
	{
		return;
	}

	int32_t _new_cnt = motor->encoder_read(motor->encoder_handle);

	memmove(motor->encoder_cnt, motor->encoder_cnt + 1, MOROT_ENCODER_CNT_NUM - 1);
	motor->encoder_cnt[MOROT_ENCODER_CNT_NUM - 1] = _new_cnt;
}

/**
 * @brief 执行目标电机的控制
 * @param motor 目标电机对象
 */
void motor_exec(struct MOTOR *motor, )
{

}

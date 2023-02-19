#include "motor_encoder.h"


/**
 * @brief 电机编码器管理结构
 */
struct MOTOR_ENCODER {
	int32_t max_cnt;      /**< 编码器计数最大值 */

	void *encoder_handle;      /**< 编码器读取对象指针 */
	MOTOR_READ_ENCODER encoder_read;      /**< 编码器值读取接口 */

	int32_t direction;      /**< 编码器采样方向 -1 反向 0 停止 1 正向 */
	int32_t encoder_cnt[MOROT_ENCODER_CNT_NUM];      /**< 电机编码器计数数组 */
};


/**
 * @brief 更新编码器采样值
 * @param encoder 目标编码器对象
 * @return int32_t 编码器本次采样增量值
 */
int32_t motor_encoder_update(struct MOTOR_ENCODER *encoder)
{
	if (NULL == encoder->encoder_read)
	{
		return 0;
	}

	int32_t _new_cnt = encoder->encoder_read(encoder->encoder_handle);



	memmove(encoder->encoder_cnt, encoder->encoder_cnt + 1, MOROT_ENCODER_CNT_NUM - 1);
	encoder->encoder_cnt[MOROT_ENCODER_CNT_NUM - 1] = _new_cnt;
}

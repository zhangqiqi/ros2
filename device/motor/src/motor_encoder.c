#include "motor_encoder.h"
#include "motor_internal.h"


/**
 * @brief 更新编码器采样值
 * @param encoder 目标编码器对象
 * @return int32_t 编码器本次采样增量值
 */
float motor_encoder_update(struct MOTOR_ENCODER *encoder)
{
	if (NULL == encoder->encoder_read)
	{
		return 0;
	}

	float _new_cnt = encoder->encoder_read(encoder->encoder_handle);

	memmove(encoder->encoder_cnt, encoder->encoder_cnt + 1, MOROT_ENCODER_CNT_NUM - 1);
	encoder->encoder_cnt[MOROT_ENCODER_CNT_NUM - 1] = _new_cnt;

	return _new_cnt;
}

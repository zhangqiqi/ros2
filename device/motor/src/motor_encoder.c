#include "motor_encoder.h"
#include "motor_internal.h"


/**
 * @brief 更新编码器采样值
 * @param encoder 目标编码器对象
 * @return int32_t 编码器本次采样增量值
 */
float motor_counter_update(struct MOTOR_COUNTER *counter)
{
	if (NULL == counter->counter_read)
	{
		return 0;
	}

	float _new_cnt = counter->counter_read(counter->counter_handle);

	memmove(counter->counter_arr, counter->counter_arr + 1, MOROT_COUNTER_CNT_NUM - 1);
	counter->counter_arr[MOROT_COUNTER_CNT_NUM - 1] = _new_cnt;

	return _new_cnt;
}


/**
 * @brief 获取计数器的最新值
 * @param encoder 目标计数器对象
 * @return 获取到的计数器的最新值
 */
float motor_counter_get_last_value(struct MOTOR_COUNTER *counter)
{
	return counter->counter_arr[MOROT_COUNTER_CNT_NUM - 1];
}

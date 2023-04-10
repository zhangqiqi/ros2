#include "counter_defs.h"
#include "counter_internal.h"
#include "counter.h"


/**
 * @brief 创建一个计数器
 * @param num 计数器记录的数据个数 num ≥ 2
 * @param max_value 创建的计数器绝对值的最大值
 * @param min_value 创建的计数器绝对值的最小值
 * @return 创建好的计数器对象 NULL 创建失败
 */
struct COUNTER *counter_create(uint32_t num, int32_t max_value, int32_t min_value)
{
	if (num < 2)
	{
		return NULL;
	}

	struct COUNTER *__new_counter = (struct COUNTER *)malloc(sizeof(struct COUNTER) + sizeof(struct COUNTER_DATA) * num);

	if (NULL != __new_counter)
	{
		__new_counter->data_num = num;
		__new_counter->max_value = max_value;
		__new_counter->min_value = min_value;
	}

	return __new_counter;
}


/**
 * @brief 以绝对值 向计数器插入新值
 * @param counter 目标计数器对象
 * @param direction 新值的变化方向
 * @param value 计数器插入新值
 * @return 0 成功 其它 失败
 */
int32_t counter_add_abs_value(struct COUNTER *counter, enum COUNTER_UPDATE_DIR direction, int32_t value)
{
	if (NULL == counter)
	{
		return -1;
	}

	memmove(counter->data, counter->data + 1, sizeof(struct COUNTER_DATA) * (counter->data_num - 1));
	counter->data[counter->data_num - 1].direction = direction;
	counter->data[counter->data_num - 1].value = value;

	return 0;
}


/**
 * @brief 以相对值 向计数器插入新值
 * @param counter 目标计数器对象
 * @param direction 新值的变化方向
 * @param value 计数器插入新值
 * @return 0 成功 其它 失败
 */
int32_t counter_add_rel_value(struct COUNTER *counter, enum COUNTER_UPDATE_DIR direction, int32_t value)
{
	/**< 把相对值转换成绝对值进行存储 自动进行极值处理 */
	int32_t abs_value = counter->data[counter->data_num - 1].value + value;
	if (abs_value > counter->max_value)
	{
		abs_value = abs_value - counter->max_value + counter->min_value;
	}
	else if (abs_value < counter->min_value)
	{
		abs_value = counter->max_value - (counter->min_value - abs_value);
	}
	else
	{
	}

	return counter_add_abs_value(counter, direction, abs_value);
}


/**
 * @brief 获取当前计数器的绝对值
 * @param counter 计数器对象
 * @param value 读取值写入
 * @return 0 成功 其它 失败
 */
int32_t counter_get_abs_value(struct COUNTER *counter, int32_t *value)
{
	if ((NULL == counter) || (NULL == value))
	{
		return -1;
	}

	*value = counter->data[counter->data_num - 1].value;
	return 0;
}


/**
 * @brief 计算两个计数器数据间的差值 一般用来计算相邻的两个数据点
 * @param counter 数据所属的计数器
 * @param data1 计数器数据1
 * @param data2 计数器数据2
 * @return 计算得到计数器间的差值
 */
static int32_t counter_get_data_diff(struct COUNTER *counter, struct COUNTER_DATA *data1, struct COUNTER_DATA *data2)
{
	int32_t ret_value = 0;

	if (CUD_FORWARD == data2->direction)
	{
		/**< 两个点之间是正向关系 */
		if (data2->value >= data1->value)
		{
			ret_value = data2->value - data1->value;
		}
		else
		{
			/**< 正向变化，但是后值比前值小，说明出现了极值跳变 */
			ret_value = (data2->value - counter->min_value) + (counter->max_value - data1->value);
		}
	}
	else if (CUD_BACKWARD == data2->direction)
	{
		/**< 两个点之间是反向关系 */
		if (data2->value <= data1->value)
		{
			ret_value = data2->value - data1->value;
		}
		else
		{
			/**< 反向变化，但后值比前值大，说明出现了极值跳变 */
			ret_value = (counter->min_value - data1->value) + (data2->value - counter->max_value);
		}
	}
	else
	{
		/**< 两个点之间关系不明确，无法计算其差值 */
	}

	return ret_value;
}


/**
 * @brief 获取当前计数器最后几个计数的相对值
 * @param counter 计数器对象
 * @param num 期望进行相对值计算的最后几个计数
 * @param value 读取值写入
 * @return 0 成功 其它 失败
 */
int32_t counter_get_rel_value(struct COUNTER *counter, int32_t num, int32_t *value)
{
	if ((NULL == counter) || (NULL == value))
	{
		return -1;
	}

	if ((num < 2) || (num > counter->data_num))
	{
		return -2;
	}

	struct COUNTER_DATA *start_pos = &counter->data[counter->data_num - num];
	int32_t ret_value = 0;

	int i = 0;
	for (i = 0; i < num - 1; i++)
	{
		ret_value += counter_get_data_diff(counter, &start_pos[i], &start_pos[i + 1]);
	}

	*value = ret_value;

	return 0;
}


/**
 * @brief 执行计数器对象 若该计数器对象是可执行对象的话
 * @return 0 成功 其它 失败
 */
int32_t counter_exec(struct COUNTER *counter)
{
	return -1;
}

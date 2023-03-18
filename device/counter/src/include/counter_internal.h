#ifndef __DEVICE_COUNTER_SRC_INCLUDE_COUNTER_INTERNAL_H__
#define __DEVICE_COUNTER_SRC_INCLUDE_COUNTER_INTERNAL_H__

#include "counter_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 计数器单点数据类型描述
 */
struct COUNTER_DATA {
	int32_t direction;      /**< 本条数据的方向 */
	int32_t value;      /**< 计数器值 */
};


struct COUNTER {
	uint32_t data_num;      /**< 本计数器管理的计数个数 */
	int32_t max_value;      /**< 计数器最大值 */
	int32_t min_value;      /**< 计数器最小值 */

	struct COUNTER_DATA data[];      /**< 计数器数据记录 按照绝对值进行记录 */
};


#ifdef __cplusplus
}
#endif

#endif //__DEVICE_COUNTER_SRC_INCLUDE_COUNTER_INTERNAL_H__

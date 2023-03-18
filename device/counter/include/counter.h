#ifndef __DEVICE_COUNTER_INCLUDE_COUNTER_H__
#define __DEVICE_COUNTER_INCLUDE_COUNTER_H__

#include "counter_defs.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief 计数器值更新方向
 */
enum COUNTER_UPDATE_DIR {
	CUD_UNKNOWN,      /**< 未知 */
	CUD_FORWARD,      /**< 正向计数 */
	CUD_BACKWARD,      /**< 反向计数 */
};

struct COUNTER;

struct COUNTER *counter_create(uint32_t num, int32_t max_value, int32_t min_value);

int32_t counter_add_abs_value(struct COUNTER *counter, enum COUNTER_UPDATE_DIR direction, int32_t value);

int32_t counter_add_rel_value(struct COUNTER *counter, enum COUNTER_UPDATE_DIR direction, int32_t value);

int32_t counter_get_abs_value(struct COUNTER *counter, int32_t *value);

int32_t counter_get_rel_value(struct COUNTER *counter, int32_t num, int32_t *value);

int32_t counter_exec(struct COUNTER *counter);

#ifdef __cplusplus
}
#endif

#endif //__DEVICE_COUNTER_INCLUDE_COUNTER_H__

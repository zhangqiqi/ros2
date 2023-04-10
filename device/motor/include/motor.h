#ifndef __DEVICE_MOTOR_INCLUDE_MOTOR_H__
#define __DEVICE_MOTOR_INCLUDE_MOTOR_H__

#include "motor_defs.h"
#include "motor_pid.h"
#include "counter.h"

#ifdef __cplusplus
extern "C" {
#endif

struct MOTOR;
struct MOTOR_COUNTER;
struct MOTOR_MANAGER;


/**
 * @brief 电机运行控制类型
 */
enum MOTOR_CTRL_TYPE {
	MCT_SPEED_RING_CTRL,      /**< 速度环控制 */
	MCT_POSITION_RING_CTRL,      /**< 位置环控制 */
};


/**
 * @brief 电机控制输出接口
 * @param ctrl_out_handle 电机控制输出操作句柄
 * @param target 输出控制目标
 * @return 0 成功 其它 失败
 */
typedef int32_t (*MOTOR_CTRL_OUT)(struct MOTOR *motor, void *ctrl_out_handle, float target);


/**
 * @brief 编码器值读取接口
 * @param handle 读编码器操作句柄
 * @return 0 更新成功 其它 失败
 */
typedef int32_t  (*MOTOR_READ_COUNTER)(struct COUNTER *counter, void *handle);


/**< 电机对象构造相关接口 */
struct MOTOR_MANAGER *motor_init(void);

struct MOTOR *motor_create(struct MOTOR_MANAGER *handle, int32_t interval_us, PIDController *pid, enum MOTOR_CTRL_TYPE type);

int32_t motor_set_counter(struct MOTOR *motor, struct COUNTER *counter);

int32_t motor_set_ctrl_out_if(struct MOTOR *motor, void *ctrl_out_handle, MOTOR_CTRL_OUT ctrl_out_if);

int32_t motor_set_counter_update_if(struct MOTOR *motor, void *counter_handle, MOTOR_READ_COUNTER update_if);


/**< 电机控制相关接口 */
void motor_set_target(struct MOTOR *motor, float target);

int32_t motor_get_target(struct MOTOR *motor);

int32_t motor_get_cur_value(struct MOTOR *motor);

float motor_get_target_ratio(struct MOTOR *motor);

void motor_state_clear(struct MOTOR *motor);

int32_t motor_exec(struct MOTOR_MANAGER *handle, int32_t elapsed_us);


#ifdef __cplusplus
}
#endif


#endif // __DEVICE_MOTOR_INCLUDE_MOTOR_H__

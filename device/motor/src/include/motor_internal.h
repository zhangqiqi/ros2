#ifndef __DEVICE_MOTOR_SRC_INCLUDE_MOTOR_INTERNAL_H__
#define __DEVICE_MOTOR_SRC_INCLUDE_MOTOR_INTERNAL_H__

#include "motor.h"
#include "motor_defs.h"
#include "queue.h"
#include "motor_pid.h"

#include <stdio.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

// #define MOTOR_DEBUG(fmt, ...) printf(fmt, ##__VA_ARGS__)
// #define MOTOR_NOTICE(fmt, ...) printf(fmt, ##__VA_ARGS__)
// #define MOTOR_ERROR(fmt, ...) printf(fmt, ##__VA_ARGS__)

#define MOTOR_DEBUG(fmt, ...)
#define MOTOR_NOTICE(fmt, ...)
#define MOTOR_ERROR(fmt, ...)

/**
 * @brief pid默认参数
 * @param pid 待初始化的pid对象
 */
#define PID_SET_DEFAULT_PARAMS(pid) \
do\
{\
	(pid)->Kp = 0.9;\
	(pid)->Ki = 0;\
	(pid)->Kd = 0;\
	(pid)->tau = 0.02;\
	(pid)->limMin = -10.0;\
	(pid)->limMax = 10.0;\
	(pid)->limMinInt = -5.0;\
	(pid)->limMaxInt = 5.0;\
	(pid)->T = 0.01;\
} while (false)


typedef void (*MOTOR_EXEC)(struct MOTOR *motor);


/**
 * @brief 电机描述结构体
 */
struct MOTOR {
	struct COUNTER *counter;      /**<电机编码计数器 */
	PIDController pid;

	int32_t target;      /**< 电机当前的目标输出值 */
	int32_t cur_value;      /**< 电机当前采样值 */

	int32_t interval_us;      /**< 电机对象的控制间隔 单位us */
	int32_t elapsed_us;      /**< 距离上次本电机被执行的时间间隔 us */

	MOTOR_EXEC exec;      /**< 电机的控制过程 */

	void *ctrl_out_handle;      /**< 电机控制输出接口操作句柄 */
	MOTOR_CTRL_OUT ctrl_out;      /**< 电机控制输出接口 */

	void *counter_update_handle;      /**< 电机计数器值更新操作句柄 */
	MOTOR_READ_COUNTER counter_update;      /**< 电机计数器值更新操作接口 */

	SIMPLEQ_ENTRY(MOTOR) MOTOR;
};

SIMPLEQ_HEAD(MOTOR_LIST, MOTOR);


/**
 * @brief 电机模块管理对象
 */
struct MOTOR_MANAGER {
	struct MOTOR_LIST motors;      /**< 管理的电机对象 */
};


#ifdef __cplusplus
}
#endif


#endif // __DEVICE_MOTOR_SRC_INCLUDE_MOTOR_INTERNAL_H__

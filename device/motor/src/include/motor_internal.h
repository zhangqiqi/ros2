#ifndef __DEVICE_MOTOR_SRC_INCLUDE_MOTOR_INTERNAL_H__
#define __DEVICE_MOTOR_SRC_INCLUDE_MOTOR_INTERNAL_H__

#include "motor.h"
#include "motor_defs.h"
#include "queue.h"
#include "PID.h"

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
	(pid)->Kp = 2;\
	(pid)->Ki = 0;\
	(pid)->Kd = 0;\
	(pid)->tau = 0.02;\
	(pid)->limMin = -10.0;\
	(pid)->limMax = 10.0;\
	(pid)->limMinInt = -5.0;\
	(pid)->limMaxInt = 5.0;\
	(pid)->T = 0.01;\
} while (false)



/**
 * @brief 电机编码器管理结构
 */
struct MOTOR_ENCODER {
	int32_t max_cnt;      /**< 编码器计数最大值 */

	void *encoder_handle;      /**< 编码器读取对象指针 */
	MOTOR_READ_ENCODER encoder_read;      /**< 编码器值读取接口 */

	int32_t direction;      /**< 编码器采样方向 -1 反向 0 停止 1 正向 */
	float encoder_cnt[MOROT_ENCODER_CNT_NUM];      /**< 电机编码器计数数组 */
};


/**
 * @brief 电机描述结构体
 */
struct MOTOR {
	struct MOTOR_ENCODER encoder;      /**< 编码器对象 */
	PIDController pid;

	float target;      /**< 电机当前的目标输出值 */

	int32_t interval_us;      /**< 电机对象的控制间隔 单位us */
	int32_t elapsed_us;      /**< 距离上次本电机被执行的时间间隔 us */

	void *ctrl_out_handle;      /**< 电机控制输出接口操作句柄 */
	MOTOR_CTRL_OUT ctrl_out;      /**< 电机控制输出接口 */

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

#ifndef __DEVICE_MOTOR_INCLUDE_MOTOR_H__
#define __DEVICE_MOTOR_INCLUDE_MOTOR_H__

#include "motor_defs.h"
#include "PID.h"

#ifdef __cplusplus
extern "C" {
#endif

struct MOTOR;

struct MOTOR_MANAGER;

struct MOTOR_MANAGER *motor_init(void);

struct MOTOR *motor_create(struct MOTOR_MANAGER *handle, int32_t interval_us, PIDController *pid);

void motor_set_target(struct MOTOR *motor, float target);

int32_t motor_exec(struct MOTOR_MANAGER *handle, int32_t elapsed_us);


/**< 电机控制相关接口 */

/**
 * @brief 电机控制输出接口
 * @param ctrl_out_handle 电机控制输出操作句柄
 * @param target 输出控制目标
 * @return 0 成功 其它 失败
 */
typedef int32_t (*MOTOR_CTRL_OUT)(void *ctrl_out_handle, float target);

int32_t motor_set_ctrl_out_if(struct MOTOR *motor, void *ctrl_out_handle, MOTOR_CTRL_OUT ctrl_out_if);

/**
 * @brief 编码器值读取接口
 * @param handle 读编码器操作句柄
 * @return 本次读取到的编码器值
 */
typedef float (*MOTOR_READ_ENCODER)(void *handle);

int32_t motor_set_encoder_read_if(struct MOTOR *motor, void *encoder_handle, MOTOR_READ_ENCODER read_if);


#ifdef __cplusplus
}
#endif


#endif // __DEVICE_MOTOR_INCLUDE_MOTOR_H__

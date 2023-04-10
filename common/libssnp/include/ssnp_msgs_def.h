#ifndef __LIBSSNP_INCLUDE_SSNP_MSGS_DEF_H__
#define __LIBSSNP_INCLUDE_SSNP_MSGS_DEF_H__

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(1)

/**
 * @brief 消息类型枚举
 */
enum SSNP_MSG_TYPE {
	SMT_UNKNOWN,      /**< 未知消息类型 */
	
	SMT_SHELL_REQ,      /**< shell请求 */
	SMT_SHELL_RES,      /**< shell响应 */

	SMT_WHEEL_MOTOR_CTRL,      /**< 轮子电机控制 */
	SMT_WHEEL_MOTOR_DATA_PUSH,      /**< 轮子电机数据推送 */
};


/**
 * @brief shell指令请求字符串
 */
struct SMT_SHELL_REQ_MSG {
	int32_t req_len;      /**< 请求消息长度 */
	char req_str[];      /**< 请求消息字符串 */
};


/**
 * @brief shell指令响应字符串
 */
struct SMT_SHELL_RES_MSG {
	int32_t res_len;      /**< 响应消息长度 */
	char res_str[];      /**< 响应消息字符串 */
};


/**
 * @brief bau轮子电机控制消息
 */
struct SMT_WHEEL_MOTOR_CTRL_MSG {
	int32_t freq;      /**< 电机控制和编码器采样频率 */
	int32_t left_motor_count;      /**< 左轮电机编码器值 */
	int32_t right_motor_count;      /**< 右轮电机编码器值 */
};


/**
 * @brief 轮子电机数据
 */
struct WHEEL_MOTOR_DATA {
	int32_t freq;      /**< 电机实际控制和编码器采样频率 */
	int32_t target_count;      /**< 目标编码器值 */
	int32_t sample_count;      /**< 采样编码器值 */
};

/**
 * @brief bau轮子电机控制数据推送
 */
struct SMT_WHEEL_MOTOR_DATA_PUSH_MSG {
	struct WHEEL_MOTOR_DATA left_motor_data;
	struct WHEEL_MOTOR_DATA right_motor_data;
};


#pragma pack()

#ifdef __cplusplus
}
#endif

#endif // __LIBSSNP_INCLUDE_SSNP_MSGS_DEF_H__

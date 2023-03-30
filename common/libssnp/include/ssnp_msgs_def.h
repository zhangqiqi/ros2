#ifndef __LIBSSNP_INCLUDE_SSNP_MSGS_DEF_H__
#define __LIBSSNP_INCLUDE_SSNP_MSGS_DEF_H__

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 消息类型枚举
 */
enum SSNP_MSG_TYPE {
	SMT_UNKNOWN,      /**< 未知消息类型 */
	
	SMT_SHELL_REQ,      /**< shell请求 */
	SMT_SHELL_RES,      /**< shell响应 */
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

#ifdef __cplusplus
}
#endif

#endif // __LIBSSNP_INCLUDE_SSNP_MSGS_DEF_H__

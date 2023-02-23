#ifndef __LIBSNP_INCLUDE_SNP_STD_MSGS_H__
#define __LIBSNP_INCLUDE_SNP_STD_MSGS_H__


#include "snp_defs.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief 协议栈标准消息枚举
 */
enum SNP_STD_MSGS_TYPE {
	SSM_UNKNOWN_MSG,      /**< 未知消息类型 */
	SSM_DISCOVERY_REQ,      /**< 设备发现请求 */
	SSM_DISCOVERY_RES,      /**< 设备发现请求响应 */

	SSM_LOG_MSG,      /**< 标准日志消息 */

	SSM_APP_MSG_FIELD_BEGIN = 1024,      /**< 应用层消息域起始枚举 应用层协议号从此开始 */
};


/**
 * @brief 设备发现请求结构
 */
struct SSM_DISCOVERY_REQ_MSG {
	char name[16];      /**< 发起请求的节点名 */
	int32_t type;      /**< 发起请求的节点类型 */
	int32_t id;      /**< 发起请求的节点标识符 */
};


/**
 * @brief 设备发现请求响应结构
 */
struct SSM_DISCOVERY_RES_MSG {
	char name[16];      /**< 发起请求的节点名 */
	int32_t type;      /**< 发起请求的节点类型 */
	int32_t id;      /**< 发起请求的节点标识符 */
};



#ifdef __cplusplus
}
#endif



#endif // __LIBSNP_INCLUDE_SNP_STD_MSGS_H__

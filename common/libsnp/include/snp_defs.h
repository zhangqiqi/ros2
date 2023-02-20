#ifndef __LIBSNP_INCLUDE_SNP_DEFS_H__
#define __LIBSNP_INCLUDE_SNP_DEFS_H__

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

struct SNP;
struct SNP_NODE;
struct SNP_LINK;
struct SNP_NODE_LIST;
struct SNP_BUFFER;
struct SNP_FRAME;
struct SNP_MSGS_PUB_LIST;

#define SNP_DEFAULT_BUFFER_SIZE (512)      /**< 默认创建协议栈每个缓存区的大小 */

typedef int32_t SNP_RET_TYPE;
#define SNP_RET_OK (0)      /**< 返回成功 */
#define SNP_RET_NULLPTR_ERR (-1)      /**< 空指针错误 */
#define SNP_RET_NO_MEM (-2)      /**< 内存不足 分配失败 */
#define SNP_RET_MSG_SEQ_ERR (-3)      /**< 消息序号错误 */

/**
 * @brief 调试信息输出枚举
 */
enum SNP_LOG_TYPE {
	SLT_DEBUG,
	SLT_NOTICE,
	SLT_ERROR
};


/**
 * @brief 协议栈设备节点类型枚举
 */
enum SNP_DEV_TYPE {
	SDT_UNKNOWN_DEV,      /**< 未知设备类型 */
	SDT_RELAY_SERVER,      /**< 消息中继服务器 */
	SDT_TIMESTAMP_SERVER,      /**< 时间戳服务器 */
	SDT_LOG_SERVER,      /**< 日志服务器 */

	SDT_ALARM_SERVER,      /**< 告警服务器 */
	SDK_BAU_MONITOR,      /**< BAU主控制板 */
};


typedef void (*SNP_LOG_IF)(enum SNP_LOG_TYPE type, char *fmt, ...);

typedef void *(*SNP_MALLOC)(uint32_t size);
typedef void (*SNP_FREE)(void *);

typedef int32_t (*SNP_LOCK)(void *handle);
typedef int32_t (*SNP_UNLOCK)(void *handle);

typedef int32_t (*SNP_SEM_WAIT)(void *handle);
typedef int32_t (*SNP_SEM_WAKE)(void *handle);


#ifdef __cplusplus
}
#endif

#endif // __LIBSNP_INCLUDE_SNP_DEFS_H__

#ifndef __LIBSSNP_INCLUDE_SSNP_H__
#define __LIBSSNP_INCLUDE_SSNP_H__

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "ssnp_buffer.h"
#include "ssnp_msgs.h"
#include "ssnp_msgs_def.h"

#ifdef __cplusplus
extern "C" {
#endif


#define SSNP_RECV_BUFFER_SIZE (1024)      /**< 接收数据缓存区大小 */
#define SSNP_TRANS_BUFFER_SIZE (1024)      /**< 发送数据缓存区大小 */

struct SSNP;

/**
 * @brief 协议栈数据接收回调类型
 * @param read_handle 接收数据操作句柄
 * @param recv_buf 接收到的数据写入缓存区
 * @return 实际接收到的数据长度 小于0则为接收异常码
 */
typedef int32_t (*SSNP_RECV_CB)(void *read_handle, struct SSNP_BUFFER *recv_buf);

/**
 * @brief 协议栈数据发送回调类型
 * @param write_handle 发送数据操作句柄
 * @param trans_buf 待发送数据缓存区
 * @return 实际发送的数据长度 小于0则为发送异常
 */
typedef int32_t (*SSNP_TRANS_CB)(void *write_handle, struct SSNP_BUFFER *trans_buf);

/**
 * @brief 协议栈日志输出接口回调类型
 * @param log_handle 日志输出操作回调
 * @param fmt 格式化日志输出字符串
 * @param ... 日志参数
 */
typedef void (*SSNP_LOG_PRINT_CB)(void *log_handle, char *fmt, ...);

/**< 协议栈构造接口 */
struct SSNP *ssnp_create(void);

int32_t ssnp_log_print_setup(void *log_handle, SSNP_LOG_PRINT_CB log_cb);

int32_t ssnp_recv_if_setup(struct SSNP *ssnp, void *recv_handle, SSNP_RECV_CB recv_cb);

int32_t ssnp_trans_if_setup(struct SSNP *ssnp, void *trans_handle, SSNP_TRANS_CB trans_cb);

int32_t ssnp_msgs_listener_setup(struct SSNP *ssnp, int32_t type, SSNP_MSG_CB msg_cb, void *cb_handle);


/**< 协议栈执行接口 */
int32_t ssnp_exec(struct SSNP *ssnp);

int32_t ssnp_send_msg(struct SSNP *ssnp, int32_t type, uint8_t *msg, int32_t len);

#ifdef __cplusplus
}
#endif

#endif  // __LIBSSNP_INCLUDE_SSNP_H__

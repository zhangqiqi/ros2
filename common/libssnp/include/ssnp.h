#ifndef __LIBSSNP_INCLUDE_SSNP_H__
#define __LIBSSNP_INCLUDE_SSNP_H__

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "ssnp_buffer.h"
#include "ssnp_msgs.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SNP_DEBUG(fmt, ...)

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

struct SSNP *ssnp_create();

int32_t ssnp_recv_if_setup(struct SSNP *ssnp, void *recv_handle, SSNP_RECV_CB recv_cb);

int32_t ssnp_trans_if_setup(struct SSNP *ssnp, void *trans_handle, SSNP_TRANS_CB trans_cb);

int32_t ssnp_exec(struct SSNP *ssnp);


#ifdef __cplusplus
}
#endif

#endif  // __LIBSSNP_INCLUDE_SSNP_H__

#ifndef __LIBSNP_INCLUDE_SNP_MSGS_H__
#define __LIBSNP_INCLUDE_SNP_MSGS_H__

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif


#pragma pack(1)

/**
 * @brief 协议栈帧结构
 */
struct SSNP_FRAME { 
	int32_t magic;      /**< 栈帧识别符 */
	int32_t frame_type;      /**< 帧消息类型 */
	int32_t frame_seq;      /**< 帧消息序列号 */
	int32_t frame_len;      /**< 帧消息负载长度 */
	int32_t crc32;      /**< 负载数据完整性校验 */
	uint8_t payload[];      /**< 负载数据域 */
};

#pragma pack()

struct SSNP;
struct SSNP_BUFFER;
struct SSNP_MSGS_PUB_LIST;

typedef int32_t (*SSNP_MSG_CB)(void *cb_handle, struct SSNP *ssnp, struct SSNP_FRAME *msg);

int32_t ssnp_msgs_unpack(struct SSNP_BUFFER *buffer, struct SSNP_FRAME **frame);

int32_t ssnp_msgs_pack(struct SSNP_BUFFER *buffer, struct SSNP_FRAME *frame, uint8_t *msg, int32_t len);

struct SSNP_MSGS_PUB_LIST *ssnp_msgs_create_pub_list(void);

int32_t ssnp_msgs_add_pub_cb(struct SSNP_MSGS_PUB_LIST *pub_list, int32_t type, SSNP_MSG_CB msg_cb, void *cb_handle);

int32_t ssnp_msgs_pub(struct SSNP_MSGS_PUB_LIST *pub_list, struct SSNP *ssnp, struct SSNP_FRAME *msg);

#ifdef __cplusplus
}
#endif


#endif // __LIBSNP_INCLUDE_SNP_MSGS_H__

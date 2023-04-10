#ifndef __LIBSNP_INCLUDE_SNP_MSGS_H__
#define __LIBSNP_INCLUDE_SNP_MSGS_H__

#include "snp_defs.h"
#include "snp_std_msgs.h"

#ifdef __cplusplus
extern "C" {
#endif


#define SNP_BROADCAST_ID (-1)      /**< 广播消息，标识需要接收到的节点，将该消息发送给该节点的所有可用连接 */
#define SNP_SINGLE_ID (0)      /**< 单连接消息，标识直接连接的两个节点的消息交互 */

#pragma pack(1)

/**
 * @brief 协议栈帧结构
 */
struct SNP_FRAME {
	int32_t magic;      /**< 栈帧识别符 */
	int32_t src_node_id;      /**< 帧源设备标识符 */
	int32_t dst_node_id;      /**< 帧目标设备标识符 SNP_BROADCAST_ID SNP_SINGLE_ID 或任意有效节点ID值 */
	int32_t frame_type;      /**< 帧消息类型 */
	int32_t frame_seq;      /**< 帧消息序列号 */
	int32_t frame_len;      /**< 帧消息负载长度 */
	int32_t crc32;      /**< 负载数据完整性校验 */
	uint8_t payload[];      /**< 负载数据域 */
};

#pragma pack()

struct SNP_MSGS_PUB_LIST;

typedef int32_t (*SNP_MSG_CB)(void *cb_handle, struct SNP_LINK *link, struct SNP_FRAME *msg);

uint32_t snp_msgs_unpack(struct SNP_BUFFER *buffer, struct SNP_FRAME **frame);

uint32_t snp_msgs_pack(struct SNP_BUFFER *buffer, struct SNP_FRAME *frame, uint8_t *msg, int32_t len);

struct SNP_MSGS_PUB_LIST *snp_msgs_create_pub_list(void);

SNP_RET_TYPE snp_msgs_add_pub_cb(struct SNP_MSGS_PUB_LIST *pub_list, int32_t type, SNP_MSG_CB msg_cb, void *cb_handle);

SNP_RET_TYPE snp_msgs_pub(struct SNP_MSGS_PUB_LIST *pub_list, struct SNP_LINK *link, struct SNP_FRAME *msg);

#ifdef __cplusplus
}
#endif


#endif // __LIBSNP_INCLUDE_SNP_MSGS_H__

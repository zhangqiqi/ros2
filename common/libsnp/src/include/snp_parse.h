#ifndef __LIBSNP_SRC_INCLUDE_SNP_PARSE_H__
#define __LIBSNP_SRC_INCLUDE_SNP_PARSE_H__


#include "snp_defs.h"
#include "snp_buffer.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief 协议栈帧结构
 * 
 */
struct SNP_FRAME {
	int32_t magic;      /**< 栈帧识别符 */
	int32_t src_node_id;      /**< 帧源设备标识符 */
	int32_t dst_node_id;      /**< 帧目标设备标识符 */
	int32_t frame_type;      /**< 帧消息类型 */
	int32_t frame_seq;      /**< 帧消息序列号 */
	int32_t frame_len;      /**< 帧消息负载长度 */
	int32_t crc32;      /**< 负载数据完整性校验 */
	uint8_t payload[];      /**< 负载数据域 */
};


uint32_t snp_proto_unpack(struct SNP_BUFFER *buffer, struct SNP_FRAME **frame);

#ifdef __cplusplus
}
#endif

#endif // __LIBSNP_SRC_INCLUDE_SNP_PARSE_H__

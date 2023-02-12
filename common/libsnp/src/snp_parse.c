#include "snp_parse.h"


/**
 * @brief 从目标缓存区中解析获取第一条完整消息
 * @param buffer 待解析处理的缓存区数据
 * @param frame 解析得到的第一条有效帧数据地址
 * @return uint32_t 本次处理后缓存区失效数据长度
 */
uint32_t snp_proto_unpack(struct SNP_BUFFER *buffer, struct SNP_FRAME **frame)
{
	uint32_t read_size = 0;


	return read_size;
}

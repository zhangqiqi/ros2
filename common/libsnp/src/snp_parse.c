#include "snp_parse.h"
#include "snp_msgs.h"

#define SNP_MSG_MAGIC (0x5AA55AA5)      /**< 默认栈帧标识符 */

/**
 * @brief 从目标缓存区中解析获取第一条完整消息
 * @param buffer 待解析处理的缓存区数据
 * @param frame 解析得到的第一条有效帧数据地址
 * @return uint32_t 本次处理后缓存区失效数据长度
 */
uint32_t snp_proto_unpack(struct SNP_BUFFER *buffer, struct SNP_FRAME **frame)
{
	uint32_t drain_size = 0;
	int32_t read_size = 0;
	struct SNP_FRAME *_frame = NULL;

	do
	{
		read_size = snp_buffer_copyout_ptr(buffer, (uint8_t **)&_frame, sizeof(struct SNP_FRAME));
		if ((NULL == _frame) || (read_size < sizeof(struct SNP_FRAME)))
		{
			/**< 数据长度不够 */
			SNP_DEBUG("%s %d: _frame is %p, buffer read size (%d), sizeof(struct SNP_FRAME) (%d)\r\n",
				__func__, __LINE__, _frame, read_size, sizeof(struct SNP_FRAME)
			);
			break;
		}

		if (SNP_MSG_MAGIC != _frame->magic)
		{
			/**< magic无法识别 移除一个无效字符 */
			SNP_DEBUG("find snp frame head magic %x failed, cur magic is %x, remove unvalid char\r\n", SNP_MSG_MAGIC, _frame->magic);
			snp_buffer_drain(buffer, 1);
			continue;
		}

		read_size = snp_buffer_copyout_ptr(buffer, (uint8_t **)&_frame, sizeof(struct SNP_FRAME) + _frame->frame_len);

		if (read_size < (sizeof(_frame) + _frame->frame_len))
		{
			/**< 完整的snp包还未接收完整 */
			SNP_DEBUG("package read size %d < request size %d, waitfor...\r\n", read_size, sizeof(_frame) + _frame->frame_len);
			break;
		}
		
		drain_size = read_size;
		*frame = _frame;
		break;
	} while (true);

	return drain_size;
}


/**
 * @brief 构造新的协议栈帧并写入到目标缓存区中
 * @param[in] buffer 待写入的缓存区对象
 * @param[in] frame 协议帧头信息
 * @param[in] msg 协议帧负载数据
 * @param[in] len 协议栈负载长度
 * @return 本次写入到缓存区中的有效数据长度
 */
uint32_t snp_proto_pack(struct SNP_BUFFER *buffer, struct SNP_FRAME *frame, uint8_t *msg, int32_t len)
{
	uint32_t write_size = 0;

	if (NULL == buffer || NULL == frame)
	{
		return 0;
	}

	frame->magic = SNP_MSG_MAGIC;
	frame->frame_len = len;
	frame->crc32 = 0;      /**< 测试阶段先不用 */

	write_size = snp_buffer_write(buffer, (uint8_t *)frame, sizeof(struct SNP_FRAME));
	write_size += snp_buffer_write(buffer, msg, len);

	return write_size;
}

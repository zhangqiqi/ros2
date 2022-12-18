#include "device_defs.h"
#include "wheeltec_n10_protocol.h"


/**
 * @brief n10帧数据和校验算法
 * @param data 进行校验的数据缓存区地址
 * @param len 进行校验的数据长度
 * @return uint8_t 计算得到的和校验值
 */
static uint8_t wheeltec_n10_checksum(uint8_t *data, int32_t len)
{
	uint8_t checksum = 0;

	while (len-- > 0)
	{
		checksum += data[len];
	}

	return checksum;
}


/**
 * @brief 按顺序从输入缓存区中，检索并校验点云帧数据
 * @param data 数据输入缓存区地址
 * @param len 数据输入缓存区数据长度
 * @param frame 解析校验通过的帧数据首地址写入地址
 * @return int32_t 本次操作完成后，输入缓存区的失效数据长度
 */
int32_t wheeltec_n10_frame_unpack(uint8_t *data, int32_t len, struct WHEELTEC_N10_FRAME **frame)
{
	int32_t read_len = 0;
	struct WHEELTEC_N10_FRAME *_frame = NULL;
	*frame = NULL;

	int i = 0;
	for (i = 0; i < len; i++)
	{
		_frame = (struct WHEELTEC_N10_FRAME *)(data + i);
		if (0x5aa5 != _frame->head)
		{
			continue;
		}

		if ((len - i) < sizeof(struct WHEELTEC_N10_FRAME))
		{
			continue;
		}

		if (wheeltec_n10_checksum(data + i, sizeof(struct WHEELTEC_N10_FRAME) - 1) != _frame->crc)
		{
			continue;
		}

		*frame = _frame;
		read_len = sizeof(struct WHEELTEC_N10_FRAME) + i;
		break;
	}

	return read_len;
}

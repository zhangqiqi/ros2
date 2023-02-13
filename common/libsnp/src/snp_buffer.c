#include "snp_buffer.h"


/**
 * @brief 缓存区管理结构体
 * 
 */
struct SNP_BUFFER {
	int32_t buffer_size;      /**< 缓存区大小 */
	int32_t read_index;      /**< 缓存区读数据索引 */
	int32_t write_index;       /**< 缓存区写数据索引 */

	uint8_t buffer[];
};


/**
 * @brief 创建新的缓存区
 * @param size 缓存区大小
 * @return struct SNP_BUFFER* 创建完成的缓存区对象指针
 */
struct SNP_BUFFER *snp_buffer_create(int32_t size)
{
	struct SNP_BUFFER *_new_buffer = (struct SNP_BUFFER *)malloc(sizeof(struct SNP_BUFFER) + size);

	if (NULL != _new_buffer)
	{
		_new_buffer->buffer_size = size;
		_new_buffer->read_index = 0;
		_new_buffer->write_index = 0;

		memset(_new_buffer->buffer, 0, _new_buffer->buffer_size);
	}

	return _new_buffer;
}


/**
 * @brief 缓存区数据写入接口
 * @param buffer 缓存区
 * @param data 待写入数据
 * @param len 待写入数据长度
 * @return int32_t 实际写入的数据长度
 */
int32_t snp_buffer_write(struct SNP_BUFFER *buffer, uint8_t *data, int32_t len)
{
	int32_t ret_len = 0;

	return ret_len;
}


/**
 * @brief 缓存区数据读取接口
 * @param buffer 缓存区
 * @param data 待读取数据写入地址
 * @param len 期望读取到的数据长度
 * @return int32_t 实际读取到的数据长度
 */
int32_t snp_buffer_read(struct SNP_BUFFER *buffer, uint8_t *data, int32_t len)
{
	int32_t ret_len = 0;

	return ret_len;
}

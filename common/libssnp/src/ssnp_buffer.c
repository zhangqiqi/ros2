#include "ssnp_buffer.h"


/**
 * @brief 获取缓存区数据长度
 */
#define BUFFER_DATA_LENGTH(buffer) ((buffer)->write_index - (buffer)->read_index)

/**
 * @brief 缓存区管理结构体
 * 
 */
struct SSNP_BUFFER {
	int32_t buffer_size;      /**< 缓存区大小 */
	int32_t read_index;      /**< 缓存区读数据索引 */
	int32_t write_index;       /**< 缓存区写数据索引 */
	uint8_t buffer[];
};


/**
 * @brief 创建新的缓存区
 * @param size 缓存区大小
 * @return struct SSNP_BUFFER* 创建完成的缓存区对象指针
 */
struct SSNP_BUFFER *ssnp_buffer_create(int32_t size)
{
	struct SSNP_BUFFER *_new_buffer = (struct SSNP_BUFFER *)malloc(sizeof(struct SSNP_BUFFER) + size);

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
 * @brief 销毁缓存区对象
 * @param buffer 待销毁的缓存区对象
 */
void ssnp_buffer_destory(struct SSNP_BUFFER *buffer)
{

}


/**
 * @brief 缓存区数据写入接口
 * @param buffer 缓存区
 * @param data 待写入数据
 * @param len 待写入数据长度
 * @return int32_t 实际写入的数据长度
 */
int32_t ssnp_buffer_write(struct SSNP_BUFFER *buffer, uint8_t *data, int32_t len)
{
	int32_t tail_size = buffer->buffer_size - buffer->write_index;
	int32_t empty_size = buffer->read_index + tail_size;

	if (empty_size < len)
	{
		/**< 剩余空间不足，不写入数据 */
		return 0;
	}

	if (tail_size < len)
	{
		/**< 尾部空间不足，整体将当前数据往缓存区头部移动，简化缓存区操作，不使用环形 */
		memmove(buffer->buffer, buffer->buffer + buffer->read_index, buffer->write_index - buffer->read_index);
		buffer->write_index -= buffer->read_index;
		buffer->read_index = 0;
	}

	memcpy(buffer->buffer + buffer->write_index, data, len);
	buffer->write_index += len;

	return len;
}


/**
 * @brief 缓存区数据读取接口
 * @param buffer 缓存区
 * @param data 待读取数据写入地址
 * @param len 期望读取到的数据长度
 * @return int32_t 实际读取到的数据长度
 */
int32_t ssnp_buffer_read(struct SSNP_BUFFER *buffer, uint8_t *data, int32_t len)
{
	int32_t ret_len = 0;
	int32_t _data_size = buffer->write_index - buffer->read_index;

	ret_len = _data_size > len ? len : _data_size;

	memcpy(data, buffer->buffer + buffer->read_index, ret_len);
	buffer->read_index += ret_len;

	return ret_len;
}


/**
 * @brief 缓存区数据读取接口 拷贝读取方式，缓存区中的数据不会被清除
 * @param buffer 缓存区
 * @param data 待读取数据写入地址
 * @param len 期望读取到的数据长度
 * @return int32_t 实际读取到的数据长度
 */
int32_t ssnp_buffer_copyout(struct SSNP_BUFFER *buffer, uint8_t *data, int32_t len)
{
	int32_t ret_len = 0;
	int32_t _data_size = buffer->write_index - buffer->read_index;

	ret_len = _data_size > len ? len : _data_size;

	memcpy(data, buffer->buffer + buffer->read_index, ret_len);

	return ret_len;
}


/**
 * @brief 缓存区数据读取接口，拷贝指针读取方式，缓存区中的数据不会被清除
 * 使用该接口需要保证，得到的指针在使用时，缓存区不会被其它地方读写
 * @param buffer 缓存区
 * @param data 拷贝指针写入地址
 * @param len 拷贝指针期望指向地址有效数据的个数
 * @return 实际拷贝指针指向的数据长度
 */
int32_t ssnp_buffer_copyout_ptr(struct SSNP_BUFFER *buffer, uint8_t **data, int32_t len)
{
	int32_t ret_len = 0;
	int32_t _data_size = buffer->write_index - buffer->read_index;

	ret_len = _data_size > len ? len : _data_size;

	*data = buffer->buffer + buffer->read_index;

	return ret_len;
}


/**
 * @brief 从一个缓存区拷贝数据到另一个缓存区
 * @param dst_buffer 目标缓存区
 * @param src_buffer 源数据缓存区
 * @return 实际拷贝的数据长度
 */
int32_t ssnp_buffer_copyout_from(struct SSNP_BUFFER *dst_buffer, struct SSNP_BUFFER *src_buffer)
{
	int32_t ret_len = 0;
	int32_t copy_size = 0;

	copy_size = BUFFER_DATA_LENGTH(src_buffer);
	ret_len = ssnp_buffer_write(dst_buffer, src_buffer->buffer + src_buffer->read_index, copy_size);

	return ret_len;
}


/**
 * @brief 从一个缓存区读取数据到另一个缓存区
 * @param dst_buffer 目标缓存区
 * @param src_buffer 源缓存区
 * @return 实际拷贝的数据长度
 */
int32_t ssnp_buffer_read_from(struct SSNP_BUFFER *dst_buffer, struct SSNP_BUFFER *src_buffer)
{
	int32_t ret_len = 0;
	int32_t copy_size = 0;

	copy_size = BUFFER_DATA_LENGTH(src_buffer);
	ret_len = ssnp_buffer_write(dst_buffer, src_buffer->buffer + src_buffer->read_index, copy_size);

	ssnp_buffer_drain(src_buffer, ret_len);

	return ret_len;
}


/**
 * @brief 缓存区数据清除
 * @param[in] buffer 缓存区指针
 * @param[in] len 清除数据长度
 * @return 实际清除的数据长度
 */
int32_t ssnp_buffer_drain(struct SSNP_BUFFER *buffer, int32_t len)
{
	int32_t _drain_size = buffer->write_index - buffer->read_index;

	_drain_size = (_drain_size > len) ? len : _drain_size;
	buffer->read_index += _drain_size;

	return _drain_size;
}
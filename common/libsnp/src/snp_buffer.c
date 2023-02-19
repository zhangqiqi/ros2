#include "snp_buffer.h"
#include "snp_defs_p.h"


/**
 * @brief 缓存区管理结构体
 * 
 */
struct SNP_BUFFER {
	SNP_LOCKER_CREATE();      /**< 创建缓存区锁对象 */

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
	SNP_LOCK(buffer);

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

	SNP_UNLOCK(buffer);

	return len;
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

	SNP_LOCK(buffer);

	int32_t _data_size = buffer->write_index - buffer->read_index;

	ret_len = _data_size > len ? len : _data_size;

	memcpy(data, buffer->buffer + buffer->read_index, ret_len);
	buffer->read_index += ret_len;

	SNP_UNLOCK(buffer);

	return ret_len;
}


/**
 * @brief 缓存区数据读取接口 拷贝读取方式，缓存区中的数据不会被清除
 * @param buffer 缓存区
 * @param data 待读取数据写入地址
 * @param len 期望读取到的数据长度
 * @return int32_t 实际读取到的数据长度
 */
int32_t snp_buffer_copyout(struct SNP_BUFFER *buffer, uint8_t *data, int32_t len)
{
	int32_t ret_len = 0;

	SNP_LOCK(buffer);

	int32_t _data_size = buffer->write_index - buffer->read_index;

	ret_len = _data_size > len ? len : _data_size;

	memcpy(data, buffer->buffer + buffer->read_index, ret_len);

	SNP_UNLOCK(buffer);

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
int32_t snp_buffer_copyout_ptr(struct SNP_BUFFER *buffer, uint8_t **data, int32_t len)
{
	int32_t ret_len = 0;

	SNP_LOCK(buffer);

	int32_t _data_size = buffer->write_index - buffer->read_index;

	ret_len = _data_size > len ? len : _data_size;

	*data = buffer->buffer + buffer->read_index;

	SNP_UNLOCK(buffer);

	return ret_len;
}


/**
 * @brief 设置缓存区锁
 * @param[in] buffer 缓存区对象指针
 * @param[in] lock_handle 锁句柄
 * @param[in] lock 加锁接口
 * @param[in] unlock 解锁接口
 * @return SNP_RET_OK 成功 其它 失败
 */
SNP_RET_TYPE snp_buffer_setup_locker(struct SNP_BUFFER *buffer, void *lock_handle, SNP_LOCK lock, SNP_UNLOCK unlock)
{
	if ((NULL == buffer) || (NULL == lock) || (NULL == unlock))
	{
		return SNP_RET_NULLPTR_ERR;
	}

	buffer->locker_handle = lock_handle;
	buffer->lock = lock;
	buffer->unlock = unlock;

	return SNP_RET_OK;
}


/**
 * @brief 缓存区数据清除
 * @param[in] buffer 缓存区指针
 * @param[in] len 清除数据长度
 * @return 实际清除的数据长度
 */
int32_t snp_buffer_drain(struct SNP_BUFFER *buffer, int32_t len)
{
	int32_t ret_len = 0;

	SNP_LOCK(buffer);

	int32_t _drain_size = buffer->write_index - buffer->read_index;
	buffer->read_index += ((_drain_size > len) ? len : _drain_size);

	SNP_UNLOCK(buffer);

	return ret_len;
}

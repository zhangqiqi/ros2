#include "snp_buffer_link.h"
#include "snp_buffer.h"
#include "snp_defs_p.h"

/**
 * @brief 缓存区连接结构体
 * 这种连接，主要用于在同一个程序的两个不同协议栈之间，建立软连接
 * 其数据交互是通过缓存区，而不是实际的通讯
 */
struct SNP_BUFFER_LINK {
	struct SNP_BUFFER *read_buffer;
	struct SNP_BUFFER *write_buffer;
};


/**
 * @brief 创建缓存区连接对象
 * @param buffer_size 对象缓存区大小
 * @return 创建成功的缓存区对象指针 NULL 失败
 */
struct SNP_BUFFER_LINK *snp_buffer_link_create(int32_t buffer_size)
{
	struct SNP_BUFFER_LINK *_new_buffer_link = (struct SNP_BUFFER_LINK *)snp_malloc(sizeof(struct SNP_BUFFER_LINK));

	memset(_new_buffer_link, 0, sizeof(struct SNP_BUFFER_LINK));
	if (NULL == _new_buffer_link)
	{
		return NULL;
	}

	_new_buffer_link->read_buffer = snp_buffer_create(buffer_size);
	if (NULL == _new_buffer_link->read_buffer)
	{
		snp_buffer_link_destory(_new_buffer_link);
		return NULL;
	}

	_new_buffer_link->write_buffer = snp_buffer_create(buffer_size);
	if (NULL == _new_buffer_link->write_buffer)
	{
		snp_buffer_destory(_new_buffer_link->read_buffer);
		snp_buffer_link_destory(_new_buffer_link);
		return NULL;
	}

	return _new_buffer_link;
}


void snp_buffer_link_destory(struct SNP_BUFFER_LINK *buffer_link)
{

}


/**
 * @brief 缓存区连接源节点读取接口 连接操作接口签名 SNP_LINK_READ
 */
int32_t snp_buffer_link_src_read(void *handle, struct SNP_BUFFER *buffer)
{
	struct SNP_BUFFER_LINK *buffer_link = (struct SNP_BUFFER_LINK *)handle;

	SNP_DEBUG("%s %d: read size %d\r\n", __func__, __LINE__, snp_buffer_read_from(buffer, buffer_link->read_buffer));

	return 0;
}


/**
 * @brief 缓存区连接源节点写入接口 连接操作接口签名 SNP_LINK_WRITE
 */
int32_t snp_buffer_link_src_write(void *handle, struct SNP_BUFFER *buffer)
{
	struct SNP_BUFFER_LINK *buffer_link = (struct SNP_BUFFER_LINK *)handle;

	SNP_DEBUG("%s %d: write size %d\r\n", __func__, __LINE__, snp_buffer_read_from(buffer_link->write_buffer, buffer));

	return 0;
}


/**
 * @brief 缓存区连接目的节点读取接口 连接操作接口签名 SNP_LINK_READ
 */
int32_t snp_buffer_link_dst_read(void *handle, struct SNP_BUFFER *buffer)
{
	struct SNP_BUFFER_LINK *buffer_link = (struct SNP_BUFFER_LINK *)handle;

	SNP_DEBUG("%s %d: read size %d\r\n", __func__, __LINE__, snp_buffer_read_from(buffer, buffer_link->write_buffer));

	return 0;
}


/**
 * @brief 缓存区连接目的节点写入接口 连接操作接口签名 SNP_LINK_WRITE
 */
int32_t snp_buffer_link_dst_write(void *handle, struct SNP_BUFFER *buffer)
{
	struct SNP_BUFFER_LINK *buffer_link = (struct SNP_BUFFER_LINK *)handle;

	SNP_DEBUG("%s %d: write size %d\r\n", __func__, __LINE__, snp_buffer_read_from(buffer_link->read_buffer, buffer));

	return 0;
}


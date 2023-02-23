#include "snp_link.h"
#include "snp_buffer.h"
#include "snp_node_internal.h"



/**
 * @brief 创建节点连接构造接口
 * @param src 连接的源节点
 * @param dst 连接的目标节点
 * @param type 创建的新连接的连接类型
 * @return struct SNP_LINK* 
 */
struct SNP_LINK *snp_link_create(struct SNP_NODE *src, struct SNP_NODE *dst, enum SNP_LINK_TYPE type)
{
	struct SNP_LINK *_new_link = NULL;

	if (NULL == src || NULL == dst)
	{
		return _new_link;
	}

	_new_link = (struct SNP_LINK *)snp_malloc(sizeof(struct SNP_LINK));
	if (NULL != _new_link)
	{
		memset(_new_link, 0, sizeof(struct SNP_LINK));
		_new_link->src_node = src;
		_new_link->dst_node = dst;
		_new_link->link_type = type;

		SNP_LOCK(src);
		LIST_INSERT_HEAD(&src->links, _new_link, LINK);
		SNP_UNLOCK(src);
	}

	return _new_link;
}


/**
 * @brief 销毁指定连接对象
 * @param link 指定连接对象指针
 */
void snp_link_destory(struct SNP_LINK *link)
{

}



/**
 * @brief 设置连接的读写接口
 * @param[in] link 待设置的连接对象
 * @param[in] read 读接口函数
 * @param[in] write 写接口函数
 * @param[in] handle 读写接口操作句柄
 * @return SNP_RET_OK 成功 其它 失败
 */
int32_t snp_link_setup_rw_cb(struct SNP_LINK *link, SNP_LINK_READ read, SNP_LINK_WRITE write, void *handle)
{
	if (NULL == link)
	{
		return SNP_RET_NULLPTR_ERR;
	}

	if (NULL != read)
	{
		/**< 初始化读接口相关资源 */
		if (NULL == link->read_buffer)
		{
			link->read_buffer = snp_buffer_create(SNP_DEFAULT_BUFFER_SIZE);
		}

		if (NULL == link->read_buffer)
		{
			return SNP_RET_NO_MEM;
		}
		link->link_read = read; 
	}

	if (NULL != write)
	{
		/**< 初始化写接口相关资源 */
		if (NULL == link->write_buffer)
		{
			link->write_buffer = snp_buffer_create(SNP_DEFAULT_BUFFER_SIZE);
		}

		if (NULL == link->write_buffer)
		{
			return SNP_RET_NO_MEM;
		}
		link->link_write = write;
	}

	link->rw_handle = handle;

	return SNP_RET_OK;
}

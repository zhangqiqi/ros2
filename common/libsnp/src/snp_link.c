#include "snp_link.h"
#include "snp_msgs.h"
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


/**
 * @brief 从指定连接发送广播消息
 * @param link 指定连接对象
 * @param msg_type 消息类型
 * @param msg 消息体
 * @param size 消息体大小
 * @return int32_t SNP_RET_OK 成功 其它 失败
 */
int32_t snp_link_send_broadcast_msg(struct SNP_LINK *link, int32_t msg_type, void *msg, int32_t size)
{
	if (NULL == link)
	{
		SNP_ERROR("%s %d: link ptr is %p\r\n", __func__, __LINE__, link);
		return SNP_RET_NULLPTR_ERR;
	}

	if (NULL == link->link_write)
	{
		return SNP_RET_OK;
	}

	struct SNP_FRAME frame = {
		.src_node_id = link->src_node->id,
		.dst_node_id = SNP_BROADCAST_ID,
		.frame_type = msg_type,
		.frame_seq = link->src_node->seq++
	};

	if (snp_msgs_pack(link->write_buffer, &frame, msg, size) > 0)
	{
		link->link_write(link->rw_handle, link->write_buffer);
	}

	return SNP_RET_OK;
}


/**
 * @brief 从指定连接发送单连接消息
 * 
 * @param link 
 * @param msg_type 
 * @param msg 
 * @param size 
 * @return int32_t 
 */
int32_t snp_link_send_single_msg(struct SNP_LINK *link, int32_t msg_type, void *msg, int32_t size)
{
	if (NULL == link)
	{
		SNP_ERROR("%s %d: link ptr is %p\r\n", __func__, __LINE__, link);
		return SNP_RET_NULLPTR_ERR;
	}

	if (NULL == link->link_write)
	{
		return SNP_RET_OK;
	}

	SNP_LOCK(link);
	struct SNP_FRAME frame = {
		.src_node_id = link->src_node->id,
		.dst_node_id = SNP_SINGLE_ID,
		.frame_type = msg_type,
		.frame_seq = link->src_node->seq++
	};

	if (snp_msgs_pack(link->write_buffer, &frame, msg, size) > 0)
	{
		link->link_write(link->rw_handle, link->write_buffer);
	}

	SNP_UNLOCK(link);

	return SNP_RET_OK;
}


/**
 * @brief 从指定连接发送定向消息
 * 
 * @param link 
 * @param msg_type 
 * @param msg 
 * @param size 
 * @return int32_t 
 */
int32_t snp_link_send_direct_msg(struct SNP_LINK *link, int32_t msg_type, void *msg, int32_t size)
{
	if (NULL == link)
	{
		SNP_ERROR("%s %d: link ptr is %p\r\n", __func__, __LINE__, link);
		return SNP_RET_NULLPTR_ERR;
	}

	if (NULL == link->link_write)
	{
		return SNP_RET_OK;
	}

	SNP_LOCK(link);
	struct SNP_FRAME frame = {
		.src_node_id = link->src_node->id,
		.dst_node_id = link->dst_node->id,
		.frame_type = msg_type,
		.frame_seq = link->src_node->seq++
	};

	if (snp_msgs_pack(link->write_buffer, &frame, msg, size) > 0)
	{
		link->link_write(link->rw_handle, link->write_buffer);
	}

	SNP_UNLOCK(link);

	return SNP_RET_OK;
}


/**
 * @brief 从指定连接转发消息
 * 
 * @param link 指定连接对象
 * @param frame 待转发的消息
 * @return int32_t SNP_RET_OK 成功 其它 失败
 */
int32_t snp_link_forward_msg(struct SNP_LINK *link, struct SNP_FRAME *frame)
{
	if (NULL == link || NULL == link->link_write)
	{
		return SNP_RET_NULLPTR_ERR;
	}

	SNP_LOCK(link);
	snp_buffer_write(link->write_buffer, (uint8_t *)frame, sizeof(struct SNP_FRAME) + frame->frame_len);
	link->link_write(link->rw_handle, link->write_buffer);
	SNP_UNLOCK(link);

	return SNP_RET_OK;
}

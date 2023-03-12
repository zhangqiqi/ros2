#include "snp_link.h"
#include "snp_msgs.h"
#include "snp_buffer.h"
#include "snp_node_internal.h"
#include "snp_std_process.h"



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

	/**< 先判断这两个节点是否已经有连接 */
	LIST_FOREACH(_new_link, &src->links, LINK)
	{
		if (dst == _new_link->dst_node)
		{
			break;
		}
	}

	if (NULL != _new_link)
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
	}
	link->link_read = read; 

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
	}
	link->link_write = write;

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
		.frame_seq = SNP_NODE_GET_NEW_SEQ(link->src_node)
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
		.frame_seq = SNP_NODE_GET_NEW_SEQ(link->src_node)
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
		.frame_seq = SNP_NODE_GET_NEW_SEQ(link->src_node)
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


/**
 * @brief 根据连接收到消息的情况，更新设备表中相关节点的信息
 * @param link 目标连接对象
 * @param frame 连接收到的消息数据
 * @return SNP_RET_OK 成功 其它 失败
 */
static SNP_RET_TYPE snp_link_stat_update(struct SNP_LINK *link, struct SNP_FRAME *frame)
{
	struct SNP_NODE *frame_src_node = NULL;

	if (frame->src_node_id == link->dst_node->id)
	{
		/**< 消息直接来源于该连接 */
		frame_src_node = link->dst_node;
	}
	else
	{
		/**< 消息来自于该连接的转发 */
		SNP_DEBUG("recv forward msg from node(%d)\r\n", frame->src_node_id);

		/**< 从当前节点向前找对应的节点 */

		frame_src_node = link->src_node;
		while (NULL != (frame_src_node = TAILQ_PREV(frame_src_node, SNP_NODE_LIST, NODE)))
		{
			if (frame->src_node_id == frame_src_node->id)
			{
				break;
			}
		}
		if (NULL == frame_src_node)
		{
			frame_src_node = link->src_node;
			while (NULL != (frame_src_node = TAILQ_NEXT(frame_src_node, NODE)))
			{
				if (frame->src_node_id == frame_src_node->id)
				{
					break;
				}
			}
		}

		if (NULL == frame_src_node)
		{
			SNP_NOTICE("The device is not discovered, dev node id: %d\r\n", frame->src_node_id);
			/**< todo 系统找不到这个节点时，应该先创建一个无连接的节点，然后去尝试请求这个节点的信息
			 * 这里还没开发好，先默认无连接的节点先成功
			 */
			// return SNP_RET_DEV_NOT_DISCOVERED;
			return SNP_RET_OK;
		}
	}

	if (frame->frame_seq <= frame_src_node->seq)
	{
		/**< 已接收并处理过的消息 */
		SNP_NOTICE("link(%p) msg from node(%p) seq err, msg seq %d <= node seq %d\r\n",
			link, frame_src_node, frame->frame_seq, frame_src_node->seq
		);
		return SNP_RET_MSG_SEQ_ERR;
	}

	frame_src_node->seq = frame->frame_seq;

	return SNP_RET_OK;
}


/**
 * @brief 转发节点消息
 * @param[in] link 消息所属的连接
 * @param[in] frame 消息帧
 * @return SNP_RET_OK 成功 其它 失败
 */
static SNP_RET_TYPE snp_link_msg_forward(struct SNP_LINK *link, struct SNP_FRAME *frame)
{
	SNP_RET_TYPE ret = 0;

	struct SNP_NODE *src_node = link->src_node;
	struct SNP_LINK *_var_link = NULL;

	LIST_FOREACH(_var_link, &src_node->links, LINK)
	{
		if (_var_link->dst_node->id != frame->dst_node_id)
		{
			continue;
		}
		/**< 有连接的目标节点，和该条消息的目标节点一致，使用该节点进行消息的传递 */
		snp_link_forward_msg(_var_link, frame);
		break;
	}

	if (NULL == _var_link)
	{
		/**< 没找到可用的连接，则转发失败 */
		ret = -1;
	}

	return ret;
}


/**
 * @brief 发布节点消息
 * @param[in] link 消息所属的连接
 * @param[in] frame 消息帧
 * @return SNP_RET_OK 成功 其它 失败
 */
static SNP_RET_TYPE snp_link_msg_publish(struct SNP_LINK *link, struct SNP_FRAME *frame)
{
	struct SNP_NODE *src_node = link->src_node;
	struct SNP_NODE *dst_node = link->dst_node;

	snp_msgs_pub(src_node->to, link, frame);
	snp_msgs_pub(dst_node->from, link, frame);

	return SNP_RET_OK;
}



/**
 * @brief 传递广播消息
 * @param[in] link 消息所属的连接
 * @param[in] frame 消息帧
 * @return SNP_RET_OK 成功 其它 失败
 */
static SNP_RET_TYPE snp_link_broadcast_msg_forward(struct SNP_LINK *link, struct SNP_FRAME *frame)
{
	struct SNP_NODE *src_node = link->src_node;
	struct SNP_LINK *_var_link = NULL;

	LIST_FOREACH(_var_link, &src_node->links, LINK)
	{
		if (_var_link == link)
		{
			/**< 跳过接收消息的这个连接 */
			continue;
		}
		/**< 有连接能够发送数据，则尝试转发该消息 */
		snp_link_forward_msg(_var_link, frame);
	}

	return SNP_RET_OK;
}


/**
 * @brief 处理节点消息
 * @param[in] link 消息所属的连接
 * @param[in] frame 消息帧
 */
static SNP_RET_TYPE snp_link_msg_proc(struct SNP_LINK *link, struct SNP_FRAME *frame)
{
	SNP_RET_TYPE ret = 0;
	struct SNP_NODE *src_node = link->src_node;

	SNP_NOTICE("recv new msg, node src: %d, node dst: %d, frame type: %d, frame seq: %d\r\n", 
		frame->src_node_id, frame->dst_node_id, frame->frame_type, frame->frame_seq
	);

	ret = snp_link_stat_update(link, frame);
	if (SNP_RET_OK != ret)
	{
		return ret;
	}

	if (SNP_SINGLE_ID == frame->dst_node_id)
	{
		/**< 本机尝试处理消息 */
		SNP_NOTICE("snp single msg type %d process...\r\n", frame->frame_type);
		ret = snp_link_msg_publish(link, frame);
		SNP_NOTICE("snp single msg process end\r\n");
	}
	else if (SNP_BROADCAST_ID == frame->dst_node_id)
	{
		/**< 广播消息，本机尝试处理消息，并转发该广播 */
		SNP_NOTICE("snp broadcast msg type %d process...\r\n", frame->frame_type);
		ret = snp_link_msg_publish(link, frame);
		ret = snp_link_broadcast_msg_forward(link, frame);
		SNP_NOTICE("snp broadcast msg process end\r\n");
	}
	else if (frame->dst_node_id == src_node->id)
	{
		/**< 消息目的地就是连接的源节点，直接进行消息的处理 */
		SNP_NOTICE("snp directed local msg type %d process...\r\n", frame->frame_type);
		ret = snp_link_msg_publish(link, frame);
		SNP_NOTICE("snp directed local msg process end\r\n");
	}
	else
	{
		/**< 消息的目的地不是源节点，需要进行转发 */
		SNP_NOTICE("snp directed msg type %d process...\r\n", frame->frame_type);
		ret = snp_link_msg_forward(link, frame);
		SNP_NOTICE("snp directed msg process end\r\n");
	}

	return ret;
}


/**
 * @brief 执行连接的消息读取
 * @param link 目标连接对象
 * @param max_msg_num 本次允许处理的最大消息条数
 * @return 本次实际处理的消息条数
 */
static int32_t snp_link_msg_read(struct SNP_LINK *link, int32_t max_msg_num)
{
	if (NULL == link->link_read)
	{
		return 0;
	}

	int proc_num = 0;

	while (proc_num < max_msg_num)
	{
		if (NULL == link->link_read)
		{
			break;
		}
		link->link_read(link->rw_handle, link->read_buffer);

		struct SNP_FRAME *_new_frame = NULL;
		uint32_t drain_size = snp_msgs_unpack(link->read_buffer, &_new_frame);
		if (NULL != _new_frame)
		{
			/**< 从连接中得到了新帧，识别并转换为系统消息，发布给对应的消息处理函数 */
			snp_link_msg_proc(link, _new_frame);

			SNP_DEBUG("link %p drain msg size: %d\r\n", link, drain_size);
			snp_buffer_drain(link->read_buffer, drain_size);
			proc_num++;
		}
		else
		{
			/**< 从连接中没有得到新帧 */
			SNP_DEBUG("link %p drain msg size: %d\r\n", link, drain_size);
			snp_buffer_drain(link->read_buffer, drain_size);
			break;
		}
	}

	return proc_num;
}


/**
 * @brief 执行snp连接的功能
 * @param link 目标snp连接对象
 */
void snp_link_exec(struct SNP_LINK *link)
{
	SNP_LOCK(link);

	/**< 处理接收缓存区中的数据 */
	snp_link_msg_read(link, SNP_LINK_PROC_MSG_MAX_NUM);

	/**< 检测连接的末端是否已识别，未识别则尝试进行识别 */
	if (SDT_UNKNOWN_DEV == link->dst_node->type)
	{
		/**< 未知设备类型，向其发起设备发现 */
		snp_link_dev_discovery(link);
	}

	/**< 处理发送缓存区中的数据 */
	if ((NULL != link->link_write) && (NULL != link->write_buffer))
	{
		link->link_write(link->rw_handle, link->write_buffer);
	}

	SNP_UNLOCK(link);
}

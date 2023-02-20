#include "snp_node.h"
#include "snp_parse.h"
#include "snp_defs_p.h"
#include "snp_msgs.h"
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
 * @brief 创建新的连接节点
 * @param[in] node_list 若非空 则将新节点挂载到该节点管理列表 
 * @param[in] name 新节点名
 * @param[in] type 新节点设备类型
 * @param[in] id 新节点设备id
 * @return struct SNP_NODE* 
 */
struct SNP_NODE *snp_node_create(struct SNP_NODE_LIST *node_list, char *name, int32_t type, int32_t id)
{
	struct SNP_NODE *_snp_node = (struct SNP_NODE *)snp_malloc(sizeof(struct SNP_NODE));

	if (NULL != _snp_node)
	{
		memset(_snp_node, 0, sizeof(struct SNP_NODE));
		
		strncpy(_snp_node->name, name, sizeof(_snp_node->name) - 1);
		_snp_node->type = type;
		_snp_node->id = id;

		_snp_node->from = snp_msgs_create_pub_list();
		_snp_node->to = snp_msgs_create_pub_list();

		LIST_INIT(&_snp_node->links);

		if (NULL != node_list)
		{
			TAILQ_INSERT_TAIL(node_list, _snp_node, NODE);
		}
	}

	return _snp_node;
}


/**
 * @brief 创建节点列表对象
 * @return struct SNP_NODE_LIST* 
 */
struct SNP_NODE_LIST *snp_node_list_create()
{
	struct SNP_NODE_LIST *_snp_node_list = (struct SNP_NODE_LIST *)snp_malloc(sizeof(struct SNP_NODE_LIST));

	if (NULL != _snp_node_list)
	{
		memset(_snp_node_list, 0, sizeof(struct SNP_NODE_LIST));
		TAILQ_INIT(_snp_node_list);
	}

	return _snp_node_list;
}


/**
 * @brief 获取节点的类型信息
 * @param node 目标节点对象
 * @return 获取到的类型信息
 */
int32_t snp_node_get_type(struct SNP_NODE *node)
{
	if (NULL == node)
	{
		return SDT_UNKNOWN_DEV;
	}

	return node->type;
}


/**
 * @brief 获取节点的标识符信息
 * @param node 目标节点对象
 * @return 获取到的标识符信息
 */
int32_t snp_node_get_id(struct SNP_NODE *node)
{
	if (NULL == node)
	{
		return SNP_RET_NULLPTR_ERR;
	}

	return node->id;
}


/**
 * @brief 获取节点的seq信息
 * @param node 目标节点
 * @return 获取到的节点seq值
 */
int32_t snp_node_get_seq(struct SNP_NODE *node)
{
	if (NULL == node)
	{
		return SNP_RET_NULLPTR_ERR;
	}

	return node->seq;
}


/**
 * @brief 获取节点名字符串
 * @param node 目标节点对象
 * @param name 节点名字符串写入地址
 * @param size 节点名写入地址区域大小
 * @return 获取到的节点名数据长度 小于0 失败
 */
int32_t snp_node_get_name(struct SNP_NODE *node, char *name, int32_t size)
{
	if (NULL == node)
	{
		return SNP_RET_NULLPTR_ERR;
	}

	strncpy(name, node->name, size - 1);
	return strlen(node->name) + 1;
}


/**
 * @brief 打印所有节点的构造信息，包括节点信息，连接信息等
 * @param node_list 
 */
void snp_nodes_print_all(struct SNP_NODE_LIST *node_list)
{
	if (NULL == node_list)
	{
		return;
	}
	struct SNP_NODE *_var_node = NULL;

	TAILQ_FOREACH(_var_node, node_list, NODE)
	{
		SNP_DEBUG("node ptr: 0x%p, name: %s, type: %d, id: %d\r\n", _var_node, _var_node->name, _var_node->type, _var_node->id);
		struct SNP_LINK *_var_link = NULL;
		LIST_FOREACH(_var_link, &_var_node->links, LINK)
		{
			SNP_DEBUG("    [node %p %s] ---%d--> [node %p %s]\r\n", 
				_var_link->src_node, _var_node->name, 
				_var_link->link_type,
				_var_link->dst_node, _var_link->dst_node->name
			);
		}
	}
}


/**
 * @brief 获取根节点
 * @param node_list 节点集合
 * @return struct SNP_NODE* 获取到的根节点对象
 */
struct SNP_NODE *snp_node_get_root(struct SNP_NODE_LIST *node_list)
{
	if (NULL == node_list)
	{
		return NULL;
	}

	return TAILQ_FIRST(node_list);
}


/**
 * @brief 设置连接的读写接口
 * @param[in] link 待设置的连接对象
 * @param[in] read 读接口函数
 * @param[in] write 写接口函数
 * @param[in] handle 读写接口操作句柄
 * @return SNP_RET_OK 成功 其它 失败
 */
SNP_RET_TYPE snp_link_setup_rw_cb(struct SNP_LINK *link, SNP_LINK_READ read, SNP_LINK_WRITE write, void *handle)
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
 * @brief 广播节点消息
 * @param[in] link 消息所属的连接
 * @param[in] frame 消息帧
 * @return SNP_RET_OK 成功 其它 失败
 */
static SNP_RET_TYPE snp_node_msg_boardcast(struct SNP_LINK *link, struct SNP_FRAME *frame)
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
		if (NULL != _var_link->link_write)
		{
			snp_buffer_write(_var_link->write_buffer, (uint8_t *)frame, sizeof(struct SNP_FRAME) + frame->frame_len);

			_var_link->link_write(_var_link->rw_handle, _var_link->write_buffer);
		}
	}

	return SNP_RET_OK;
}


/**
 * @brief 转发节点消息
 * @param[in] link 消息所属的连接
 * @param[in] frame 消息帧
 * @return SNP_RET_OK 成功 其它 失败
 */
static SNP_RET_TYPE snp_node_msg_forward(struct SNP_LINK *link, struct SNP_FRAME *frame)
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
		if (NULL != _var_link->link_write)
		{
			snp_buffer_write(_var_link->write_buffer, (uint8_t *)frame, sizeof(struct SNP_FRAME) + frame->frame_len);

			_var_link->link_write(_var_link->rw_handle, _var_link->write_buffer);
			break;
		}
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
static SNP_RET_TYPE snp_node_msg_publish(struct SNP_LINK *link, struct SNP_FRAME *frame)
{
	struct SNP_NODE *src_node = link->src_node;
	struct SNP_NODE *dst_node = link->dst_node;

	snp_msgs_pub(src_node->to, link, frame);
	snp_msgs_pub(dst_node->from, link, frame);

	return SNP_RET_OK;
}


/**
 * @brief 根据连接收到消息的情况，更新设备表中相关节点的信息
 * @param link 目标连接对象
 * @param frame 连接收到的消息数据
 * @return SNP_RET_OK 成功 其它 失败
 */
static SNP_RET_TYPE snp_node_stat_update(struct SNP_LINK *link, struct SNP_FRAME *frame)
{
	struct SNP_NODE *src_node = link->src_node;
	struct SNP_NODE *dst_node = link->dst_node;

	if (frame->frame_seq <= dst_node->seq)
	{
		/**< 已接收并处理过的消息 */
		SNP_NOTICE("link(%p) msg from node(%p) seq err, msg seq %d <= node seq %d\r\n",
			link, dst_node, frame->frame_seq, dst_node->seq
		);
		return SNP_RET_MSG_SEQ_ERR;
	}

	dst_node->seq = frame->frame_seq;

	return SNP_RET_OK;
}


/**
 * @brief 处理节点消息
 * @param[in] link 消息所属的连接
 * @param[in] frame 消息帧
 */
static SNP_RET_TYPE snp_node_msg_proc(struct SNP_LINK *link, struct SNP_FRAME *frame)
{
	SNP_RET_TYPE ret = 0;

	struct SNP_NODE *src_node = link->src_node;
	struct SNP_NODE *dst_node = link->dst_node;

	SNP_DEBUG("recv new msg, node src: %d, node dst: %d, frame type: %d\r\n", 
		frame->src_node_id, frame->dst_node_id, frame->frame_type
	);

	ret = snp_node_stat_update(link, frame);
	if (SNP_RET_OK != ret)
	{
		return ret;
	}

	if (SNP_SINGLE_ID == frame->dst_node_id)
	{
		/**< 本机尝试处理消息 */
		ret = snp_node_msg_publish(link, frame);
	}
	else if (SNP_BOARDCAST_ID == frame->dst_node_id)
	{
		/**< 广播消息，本机尝试处理消息，并转发该广播 */
		ret = snp_node_msg_publish(link, frame);
		ret = snp_node_msg_boardcast(link, frame);
	}
	else if (frame->dst_node_id == src_node->id)
	{
		/**< 消息目的地就是连接的源节点，直接进行消息的处理 */
		ret = snp_node_msg_publish(link, frame);
	}
	else
	{
		/**< 消息的目的地不是源节点，需要进行转发 */
		ret = snp_node_msg_forward(link, frame);
	}

	return ret;
}


/**
 * @brief 执行snp连接的功能
 * @param link 目标snp连接对象
 */
static void snp_link_exec(struct SNP_LINK *link)
{
	struct SNP_FRAME *_new_frame = NULL;

	SNP_LOCK(link);
	if (NULL != link->link_read)
	{
		link->link_read(link->rw_handle, link->read_buffer);

		uint32_t drain_size = snp_proto_unpack(link->read_buffer, &_new_frame);
		if (NULL != _new_frame)
		{
			/**< 从连接中得到了新帧，识别并转换为系统消息，发布给对应的消息处理函数 */
			snp_node_msg_proc(link, _new_frame);
		}
		SNP_DEBUG("link %p drain msg size: %d\r\n", link, drain_size);
		snp_buffer_drain(link->read_buffer, drain_size);
	}

	/**< 检测连接的末端是否已识别，未识别则尝试进行识别 */
	if (SDT_UNKNOWN_DEV == link->dst_node->type)
	{
		/**< 未知设备类型，向其发起设备发现 */
		snp_node_dev_discovery(link);
	}

	SNP_UNLOCK(link);
}


/**
 * @brief 执行当前可用节点 实际上是遍历执行主节点的所有可用连接，从这些连接上获取数据
 * @param node_list 节点列表 
 */
void snp_node_exec(struct SNP_NODE_LIST *node_list)
{
	struct SNP_LINK *_var_link = NULL;

	struct SNP_NODE *_exec_node = TAILQ_FIRST(node_list);

	/**< 遍历根节点的所有可读连接，执行连接功能 */
	SNP_LOCK(_exec_node);
	LIST_FOREACH(_var_link, &_exec_node->links, LINK)
	{
		snp_link_exec(_var_link);
	}
	SNP_UNLOCK(_exec_node);
}



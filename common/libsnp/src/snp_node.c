#include "snp_node.h"
#include "snp_defs_p.h"
#include "snp_msgs.h"
#include "snp_node_internal.h"
#include "snp_std_process.h"
#include "snp_buffer.h"
#include "snp_internal.h"



/**
 * @brief 创建新的连接节点
 * @param[in] snp 节点所属的协议栈
 * @param[in] node_list 若非空 则将新节点挂载到该节点管理列表 
 * @param[in] name 新节点名
 * @param[in] type 新节点设备类型
 * @param[in] id 新节点设备id
 * @return struct SNP_NODE* 
 */
struct SNP_NODE *snp_node_create(struct SNP *snp, struct SNP_NODE_LIST *node_list, char *name, int32_t type, int32_t id)
{
	struct SNP_NODE *_snp_node = snp_internal_get_new_node();

	if (NULL != _snp_node)
	{
		memset(_snp_node, 0, sizeof(struct SNP_NODE));
		
		_snp_node->snp = snp;

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
 * @brief 节点销毁接口 销毁节点时，会销毁协议栈内，与这个节点有关的所有连接
 */
void snp_node_destory(struct SNP_NODE *node)
{

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
 * @brief 获取源节点和目标节点的可用连接路径，通过目标节点对象搜索
 * @param src 源节点
 * @param dst 目的节点
 * @return 得到的可用连接 NULL 目标节点不可达
 */
struct SNP_LINK *snp_link_get_by_node(struct SNP_NODE *src_node, struct SNP_NODE *dst)
{
	struct SNP_LINK *_link = NULL;
	SNP_LOCK(src_node);

	LIST_FOREACH(_link, &src_node->links, LINK)
	{
		if (dst == _link->dst_node)
		{
			break;
		}
	}

	SNP_UNLOCK(src_node);

	return _link;
}


/**
 * @brief 获取源节点和目标节点的可用连接路径，通过目标节点名搜索
 * @param src 源节点
 * @param name 目标节点名
 * @return 得到的可用连接 NULL 目标节点不可
 */
struct SNP_LINK *snp_link_get_by_name(struct SNP_NODE *src_node, char *name)
{
	struct SNP_LINK *_link = NULL;
	SNP_LOCK(src_node);

	LIST_FOREACH(_link, &src_node->links, LINK)
	{
		if (0 == strcmp(_link->dst_node->name, name))
		{
			break;
		}
	}

	SNP_UNLOCK(src_node);

	return _link;
}


/**
 * @brief 获取源节点和目标节点的可用连接路径，通过目标节点id搜索
 * @param src 源节点
 * @param id 目标节点id
 * @return 得到的可用连接 NULL 目标节点不可
 */
struct SNP_LINK *snp_link_get_by_id(struct SNP_NODE *src_node, int32_t id)
{
	struct SNP_LINK *_link = NULL;
	SNP_LOCK(src_node);

	LIST_FOREACH(_link, &src_node->links, LINK)
	{
		if (id == _link->dst_node->id)
		{
			break;
		}
	}

	SNP_UNLOCK(src_node);

	return _link;
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
		SNP_NOTICE("node ptr: %p, name: %s, type: %d, id: %d\r\n", _var_node, _var_node->name, _var_node->type, _var_node->id);
		struct SNP_LINK *_var_link = NULL;
		LIST_FOREACH(_var_link, &_var_node->links, LINK)
		{
			SNP_NOTICE("    [node %p %s id:%d] ---%d--> [node %p %s id:%d]\r\n", 
				_var_link->src_node, _var_node->name, _var_node->id,
				_var_link->link_type,
				_var_link->dst_node, _var_link->dst_node->name, _var_link->dst_node->id
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
 * @brief 从节点集合中，获取指定节点对象
 * @param node_list 节点集合
 * @param id 目标节点id
 * @return 得到的目标节点对象 NULL 获取失败，节点不存在
 */
struct SNP_NODE *snp_node_get_by_id(struct SNP_NODE_LIST *node_list, int32_t id)
{
	struct SNP_NODE *_var_node = NULL;

	TAILQ_FOREACH(_var_node, node_list, NODE)
	{
		if (id == _var_node->id)
		{
			break;
		}
	}

	return _var_node;
}


/**
 * @brief 通过连接发送消息
 * @param link 目标连接
 * @param msg_type 消息类型
 * @param msg 消息数据
 * @param size 消息数据长度
 * @return 发送成功的数据长度
 */
int32_t snp_link_write(struct SNP_LINK *link, int32_t msg_type, void *msg, int32_t size)
{
	if (NULL == link || NULL == link->link_write)
	{
		return 0;
	}

	struct SNP_FRAME frame = {
		.src_node_id = link->src_node->id,
		.dst_node_id = link->dst_node->id,
		.frame_type = msg_type,
		.frame_seq = link->src_node->seq++
	};

	if (NULL != link->write_buffer)
	{
		snp_msgs_pack(link->write_buffer, &frame, (uint8_t *)msg, size);
		return link->link_write(link->rw_handle, link->write_buffer);
	}
	
	return 0;
}


/**
 * @brief 从指定节点广播一条消息
 * @param node 指定节点对象
 * @param msg_type 消息类型
 * @param msg 消息体
 * @param size 消息体大小
 * @return int32_t SNP_RET_OK 成功 其它 失败
 */
int32_t snp_node_broadcast_msg(struct SNP_NODE *node, int32_t msg_type, void *msg, int32_t size)
{
	struct SNP_LINK *_link = NULL;

	if (NULL == node)
	{
		return SNP_RET_NULLPTR_ERR;
	}

	SNP_LOCK(node);

	LIST_FOREACH(_link, &node->links, LINK)
	{
		snp_link_send_broadcast_msg(_link, msg_type, msg, size);
	}

	SNP_UNLOCK(node);

	return SNP_RET_OK;
}


/**
 * @brief 监听来自于指定节点的消息
 * @param node 指定节点对象
 * @param msg_type 消息类型
 * @param msg_cb 接收消息回调
 * @param cb_handle 接收消息回调的操作句柄
 * @return 0 成功 其它 失败
 */
int32_t snp_node_listen_msg_from(struct SNP_NODE *node, int32_t msg_type, SNP_MSG_CB msg_cb, void *cb_handle)
{
	return snp_msgs_add_pub_cb(node->from, msg_type, msg_cb, cb_handle);
}


/**
 * @brief 监听发送到指定节点的消息
 * @param node 指定节点对象
 * @param msg_type 消息类型
 * @param msg_cb 接收消息回调
 * @param cb_handle 接收消息回调的操作句柄
 * @return 0 成功 其它 失败
 */
int32_t snp_node_listen_msg_to(struct SNP_NODE *node, int32_t msg_type, SNP_MSG_CB msg_cb, void *cb_handle)
{
	return snp_msgs_add_pub_cb(node->to, msg_type, msg_cb, cb_handle);
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


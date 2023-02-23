#include "snp_std_process.h"
#include "snp_defs_p.h"
#include "snp_msgs.h"
#include "snp_std_msgs.h"
#include "snp_node_internal.h"


/**
 * @brief 尝试发现指定连接的终端设备信息
 * @param[in] link 节点连接对象
 * @return 0 成功 其它 失败
 */
int32_t snp_node_dev_discovery(struct SNP_LINK *link)
{
	if (NULL == link->link_write)
	{
		return -1;
	}

	struct SNP_FRAME frame = {
		.src_node_id = link->src_node->id,
		.dst_node_id = SNP_SINGLE_ID,
		.frame_type = SSM_DISCOVERY_REQ,
		.frame_seq = link->src_node->seq
		};
	struct SSM_DISCOVERY_REQ_MSG discovery_req_msg = {
		.type = link->src_node->type,
		.id = link->src_node->id
	};

	strncpy(discovery_req_msg.name, link->src_node->name, sizeof(discovery_req_msg.name) - 1);

	snp_msgs_pack(link->write_buffer, &frame, (uint8_t *)&discovery_req_msg, sizeof(discovery_req_msg));

	link->link_write(link->rw_handle, link->write_buffer);

	SNP_DEBUG("link %p send discovery request from node(id: %d, type: %d, name: %s) to node(id: %d, type: %d, name: %s\r\n", 
		link, 
		link->src_node->id, link->src_node->type, link->src_node->name, 
		link->dst_node->id, link->dst_node->type, link->dst_node->name
	);

	return 0;
}


/**
 * @brief 接收设备发现请求消息
 * @param cb_handle 接收响应注册使用的应用句柄
 * @param link 接收到消息的连接对象
 * @param msg 接收到的消息体
 * @return 0 成功 其它 失败
 */
static int32_t snp_node_dev_discovery_req_msg_proc(void *cb_handle, struct SNP_LINK *link, struct SNP_FRAME *msg)
{
	struct SNP_FRAME frame = {0};
	struct SSM_DISCOVERY_RES_MSG res_msg = {0};

	frame.src_node_id = link->src_node->id;
	frame.dst_node_id = msg->src_node_id;
	frame.frame_type = SSM_DISCOVERY_RES;
	frame.frame_seq = link->src_node->seq++;

	res_msg.type = link->src_node->type;
	res_msg.id = link->src_node->id;
	strncpy(res_msg.name, link->src_node->name, sizeof(res_msg.name) - 1);

	snp_msgs_pack(link->write_buffer, &frame, (uint8_t *)&res_msg, sizeof(res_msg));

	return 0;
}


/**
 * @brief 接收连接的设备发现协议响应消息
 * @param cb_handle 接收响应注册使用的应用句柄
 * @param link 接收到消息的连接对象
 * @param msg 接收到的消息体
 * @return 0 成功 其它 失败
 */
static int32_t snp_node_dev_discovery_res_msg_proc(void *cb_handle, struct SNP_LINK *link, struct SNP_FRAME *msg)
{
	struct SNP_NODE *_dst_node = link->dst_node;

	struct SSM_DISCOVERY_RES_MSG *_msg = (struct SSM_DISCOVERY_RES_MSG *)msg->payload;

	_dst_node->type = _msg->type;
	_dst_node->id = _msg->id;

	strncpy(_dst_node->name, _msg->name, sizeof(_dst_node->name));

	SNP_DEBUG("new node(%p) info update: type %d, id %d, name %s\r\n",
		_dst_node, _dst_node->type, _dst_node->id, _dst_node->name
	);
	return 0;
}


/**
 * @brief 向目标节点注册标准消息处理过程
 * @param node 目标节点对象
 * @return 0 成功 其它 失败
 */
int32_t snp_std_process_setup(struct SNP_NODE *node)
{
	if (NULL == node || NULL == node->from || NULL == node->to)
	{
		SNP_DEBUG("node %p setup std process failed\r\n", node);
		return SNP_RET_NULLPTR_ERR;
	}

	snp_msgs_add_pub_cb(node->to, SSM_DISCOVERY_REQ, snp_node_dev_discovery_req_msg_proc, node);
	snp_msgs_add_pub_cb(node->to, SSM_DISCOVERY_RES, snp_node_dev_discovery_res_msg_proc, node);
}

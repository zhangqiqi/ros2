#ifndef __LIBSNP_SRC_INCLUDE_SNP_NODE_INTERNAL_H__
#define __LIBSNP_SRC_INCLUDE_SNP_NODE_INTERNAL_H__

#include "snp_defs.h"
#include "snp_defs_p.h"
#include "snp_node.h"
#include "snp_link.h"

#include "snp_queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 获取节点的可用seq，每次获取后，该seq都递增
 */
#define SNP_NODE_GET_NEW_SEQ(node) (((node)->seq++ <= 0) ? 1 : (node)->seq)


/**
 * @brief @brief 系统节点连接描述结构
 */
struct SNP_LINK {
	SNP_LOCKER_CREATE();

	enum SNP_LINK_TYPE link_type;      /**< 连接的类型 */

	struct SNP_NODE *src_node;      /**< 一个连接的源节点 */
	struct SNP_NODE *dst_node;      /**< 一个连接的目标节点 */

	void *rw_handle;      /**< 读写消息句柄 */
	struct SNP_BUFFER *read_buffer;      /**< 连接读缓存区 */
	struct SNP_BUFFER *write_buffer;      /**< 连接写缓存区 */
	SNP_LINK_READ link_read;      /**< 连接的数据读取接口 */
	SNP_LINK_WRITE link_write;      /**< 连接的数据写入接口 */

	LIST_ENTRY(SNP_LINK) LINK;
};

LIST_HEAD(SNP_LINK_LIST, SNP_LINK);


/**
 * @brief 系统节点描述结构
 */
struct SNP_NODE
{
	SNP_LOCKER_CREATE();

	struct SNP *snp;      /**< 节点所属的协议栈对象 */

	char name[SNP_NODE_NAME_SIZE];      /**< 节点设备名 */
	int32_t type;      /**< 节点设备类型 */
	int32_t id;      /**< 节点标识符 */
	int32_t seq;      /**< 节点消息序列号 每次发送一条从该节点发布的消息，seq需要 +1 */

	struct SNP_MSGS_PUB_LIST *from;      /**< 监听来自于这个节点的消息 */
	struct SNP_MSGS_PUB_LIST *to;      /**< 监听发送到该节点的消息 */

	struct SNP_LINK_LIST links;      /**< 节点支持的外部连接 */

	TAILQ_ENTRY(SNP_NODE) NODE;
};


TAILQ_HEAD(SNP_NODE_LIST, SNP_NODE);


/**
 * @brief 协议栈管理结构
 */
struct SNP {
	SNP_LOCKER_CREATE();

	uint32_t snp_tick;      /**< 协议栈的运行滴答，每次协议栈运行时更新，精度需要应用层保证 */
	uint32_t snp_network_sync_tick;      /**< 协议栈组网信息同步滴答，倒计时方式 精度需要应用层保证 */
	struct SNP_NODE_LIST *nodes;
};


/**
 * @brief snp消息发布管理结构体
 */
struct SNP_MSGS_PUB {
	int32_t type;      /**< 发布消息类型 */

	void *cb_handle;      /**< 监听发布消息的回调句柄 */
	SNP_MSG_CB cb;      /**< 消息发布回调 */

	TAILQ_ENTRY(SNP_MSGS_PUB) MSGS;
};

TAILQ_HEAD(SNP_MSGS_PUB_LIST, SNP_MSGS_PUB);


#ifdef __cplusplus
}
#endif

#endif // __LIBSNP_SRC_INCLUDE_SNP_NODE_INTERNAL_H__

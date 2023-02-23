#ifndef __LIBSNP_SRC_INCLUDE_SNP_NODE_INTERNAL_H__
#define __LIBSNP_SRC_INCLUDE_SNP_NODE_INTERNAL_H__

#include "snp_defs.h"
#include "snp_defs_p.h"
#include "snp_node.h"
#include "snp_link.h"

#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif


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

	char name[32];      /**< 节点设备名 */
	int32_t type;      /**< 节点设备类型 */
	int32_t id;      /**< 节点标识符 */
	int32_t seq;      /**< 节点消息序列号 每次发送一条从该节点发布的消息，seq需要 +1 */

	struct SNP_MSGS_PUB_LIST *from;      /**< 监听来自于这个节点的消息 */
	struct SNP_MSGS_PUB_LIST *to;      /**< 监听发送到该节点的消息 */

	struct SNP_LINK_LIST links;      /**< 节点支持的外部连接 */

	TAILQ_ENTRY(SNP_NODE) NODE;
};


TAILQ_HEAD(SNP_NODE_LIST, SNP_NODE);


#ifdef __cplusplus
}
#endif

#endif // __LIBSNP_SRC_INCLUDE_SNP_NODE_INTERNAL_H__

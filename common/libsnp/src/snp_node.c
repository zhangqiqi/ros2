#include "snp_node.h"

#include "queue.h"


/**
 * @brief @brief 系统节点连接描述结构
 */
struct SNP_LINK {
	struct SNP_NODE *src_node;      /**< 一个连接的源节点 */
	struct SNP_NODE *dst_node;      /**< 一个连接的目标节点 */

	void *link_handle;
	SNP_NODE_READ link_read;      /**< 节点数据读取接口 */
	SNP_NODE_WRITE link_write;      /**< 节点数据写入接口 */

	LIST_ENTRY(SNP_LINK) LINK;
};

LIST_HEAD(SNP_LINK_LIST, SNP_LINK);


/**
 * @brief 系统节点描述结构
 */
struct SNP_NODE
{

	struct SNP_LINK_LIST links;
};


/**
 * @brief 创建新的连接节点
 * 
 * @return struct SNP_NODE* 
 */
struct SNP_NODE *snp_node_create()
{
	struct SNP_NODE *_snp_node = (struct SNP_NODE *)malloc(sizeof(struct SNP_NODE));

	if (NULL != _snp_node)
	{
		LIST_INIT(&_snp_node->links);
	}

	return _snp_node;
}


#ifndef __LIBSNP_INCLUDE_SNP_NODE_H__
#define __LIBSNP_INCLUDE_SNP_NODE_H__


#include "snp_defs.h"


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 连接对象类型
 */
enum SNP_LINK_TYPE {
	SLT_UNKNOWN,      /**< 未知类型 */
	SLT_PHYSICAL_LINK,      /**< 物理连接 */
	SLT_SOFTWARE_LINK,      /**< 软件连接 */
	SLT_VIRTUAL_LINK,      /**< 虚拟连接 */
};


struct SNP_NODE;
struct SNP_NODE_LIST;

typedef int32_t (*SNP_LINK_READ)(void *handle, struct SNP_BUFFER *buffer);
typedef int32_t (*SNP_LINK_WRITE)(void *handle, struct SNP_BUFFER *buffer);

struct SNP_NODE_LIST *snp_node_list_create();

void snp_node_exec(struct SNP_NODE_LIST *node_list);

/**< 节点访问相关节点 */
struct SNP_NODE *snp_node_get_root(struct SNP_NODE_LIST *node_list);

int32_t snp_node_get_type(struct SNP_NODE *node);

int32_t snp_node_get_id(struct SNP_NODE *node);

int32_t snp_node_get_seq(struct SNP_NODE *node);

int32_t snp_node_get_name(struct SNP_NODE *node, char *name, int32_t size);

/**< 节点构造相关接口 */
struct SNP_NODE *snp_node_create(struct SNP_NODE_LIST *node_list, char *name, int32_t type, int32_t id);

/**< 连接构造相关接口 */
struct SNP_LINK *snp_link_create(struct SNP_NODE *src, struct SNP_NODE *dst, enum SNP_LINK_TYPE type);

SNP_RET_TYPE snp_link_setup_rw_cb(struct SNP_LINK *link, SNP_LINK_READ read, SNP_LINK_WRITE write, void *handle);

/**< 节点调测相关接口 */
void snp_nodes_print_all(struct SNP_NODE_LIST *node_list);


#ifdef __cplusplus
}
#endif

#endif // __LIBSNP_INCLUDE_SNP_NODE_H__

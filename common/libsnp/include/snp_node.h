#ifndef __LIBSNP_INCLUDE_SNP_NODE_H__
#define __LIBSNP_INCLUDE_SNP_NODE_H__


#include "snp_defs.h"


#ifdef __cplusplus
extern "C" {
#endif

struct SNP_NODE;
struct SNP_NODE_LIST;

typedef int32_t (*SNP_LINK_READ)(void *handle, struct SNP_BUFFER *buffer);
typedef int32_t (*SNP_LINK_WRITE)(void *handle, struct SNP_BUFFER *buffer);

struct SNP_NODE_LIST *snp_node_list_create();

void snp_node_exec(struct SNP_NODE_LIST *node_list);

/**< 节点访问相关节点 */
struct SNP_NODE *snp_node_get_root(struct SNP_NODE_LIST *node_list);

/**< 节点构造相关接口 */
struct SNP_NODE *snp_node_create(struct SNP_NODE_LIST *node_list);

/**< 连接构造相关接口 */
struct SNP_LINK *snp_link_create(struct SNP_NODE *src, struct SNP_NODE *dst);

/**< 节点调测相关接口 */
void snp_nodes_print_all(struct SNP_NODE_LIST *node_list);


#ifdef __cplusplus
}
#endif

#endif // __LIBSNP_INCLUDE_SNP_NODE_H__

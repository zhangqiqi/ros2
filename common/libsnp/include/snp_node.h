#ifndef __LIBSNP_INCLUDE_SNP_NODE_H__
#define __LIBSNP_INCLUDE_SNP_NODE_H__


#include "snp_defs.h"
#include "snp_msgs.h"


#ifdef __cplusplus
extern "C" {
#endif


struct SNP_NODE;
struct SNP_NODE_LIST;

struct SNP_NODE_LIST *snp_node_list_create(void);

struct SNP_NODE *snp_node_create(struct SNP *snp, struct SNP_NODE_LIST *node_list, char *name, int32_t type, int32_t id);

void snp_node_destory(struct SNP_NODE *node);

void snp_node_exec(struct SNP_NODE_LIST *node_list);

int32_t snp_node_broadcast_msg(struct SNP_NODE *node, int32_t msg_type, void *msg, int32_t size);

int32_t snp_node_listen_msg_from(struct SNP_NODE *node, int32_t msg_type, SNP_MSG_CB msg_cb, void *cb_handle);

int32_t snp_node_listen_msg_to(struct SNP_NODE *node, int32_t msg_type, SNP_MSG_CB msg_cb, void *cb_handle);

/**< 节点访问相关节点 */
struct SNP_NODE *snp_node_get_root(struct SNP_NODE_LIST *node_list);

struct SNP_NODE *snp_node_get_by_id(struct SNP_NODE_LIST *node_list, int32_t id);

struct SNP_LINK *snp_link_get_by_node(struct SNP_NODE *src_node, struct SNP_NODE *dst);

struct SNP_LINK *snp_link_get_by_name(struct SNP_NODE *src_node, char *name);

struct SNP_LINK *snp_link_get_by_id(struct SNP_NODE *src_node, int32_t id);

int32_t snp_link_write(struct SNP_LINK *link, int32_t msg_type, void *msg, int32_t size);


/**< 节点调测相关接口 */
void snp_nodes_print_all(struct SNP_NODE_LIST *node_list);


#ifdef __cplusplus
}
#endif

#endif // __LIBSNP_INCLUDE_SNP_NODE_H__

#ifndef __LIBSNP_SRC_INCLUDE_SNP_INTERNAL_H__
#define __LIBSNP_SRC_INCLUDE_SNP_INTERNAL_H__

#include "snp.h"
#include "snp_node_internal.h"


#ifdef __cplusplus
extern "C" {
#endif

#define SNP_RESOURCE_PRE_MALLOC

#ifdef SNP_RESOURCE_PRE_MALLOC
	#define SNP_LINK_PRE_MALLOC_NUM (10)      /**< 预分配连接资源个数 */
	#define SNP_NODE_PRE_MALLOC_NUM (20)      /**< 预分配节点资源个数 */
	#define SNP_MSGS_PUB_PRE_MALLOC_NUM (20)      /**< 预分配消息监听资源 */
#endif

void snp_internal_init(void);

struct SNP_NODE *snp_internal_get_new_node(void);

void snp_internal_release_node(struct SNP_NODE *node);

struct SNP_LINK *snp_internal_get_new_link(void);

void snp_internal_release_link(struct SNP_LINK *link);

struct SNP_MSGS_PUB *snp_internal_get_new_msgs_pub(void);

void snp_internal_release_msgs_pub(struct SNP_MSGS_PUB *msgs_pub);

#ifdef __cplusplus
}
#endif

#endif // __LIBSNP_SRC_INCLUDE_SNP_INTERNAL_H__


#ifndef __LIBSNP_INCLUDE_SNP_H__
#define __LIBSNP_INCLUDE_SNP_H__

#include "snp_defs.h"
#include "snp_node.h"
#include "snp_link.h"

#ifdef __cplusplus
extern "C" {
#endif

struct SNP;

struct SNP *snp_create(char *name, int32_t type, int32_t id);

struct SNP_NODE *snp_create_physical_node(struct SNP *handle, SNP_LINK_READ read, SNP_LINK_WRITE write, void *rw_handle);

SNP_RET_TYPE snp_exec(struct SNP *handle, int32_t elapsed_ms);

struct SNP_NODE *snp_get_local_node(struct SNP *handle);

int32_t snp_send_msg_by_node(struct SNP *handle, struct SNP_NODE *dst_node, int32_t msg_type, void *msg, int32_t size);

int32_t snp_send_msg_by_name(struct SNP *handle, char *name, int32_t msg_type, void *msg, int32_t size);

int32_t snp_send_msg_by_id(struct SNP *handle, int32_t id, int32_t msg_type, void *msg, int32_t size);

int32_t snp_broadcast_msg(struct SNP *handle, int32_t msg_type, void *msg, int32_t size);

/**< 协议栈通用配置接口 */
SNP_RET_TYPE snp_set_log_if(SNP_LOG_IF log_if);

SNP_RET_TYPE snp_set_mem_if(SNP_MALLOC malloc_if, SNP_FREE free_if);

void snp_print_all(struct SNP *handle);

#ifdef __cplusplus
}
#endif

#endif // __LIBSNP_INCLUDE_SNP_H__

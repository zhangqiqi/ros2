#ifndef __LIBSNP_INCLUDE_SNP_H__
#define __LIBSNP_INCLUDE_SNP_H__

#include "snp_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

struct SNP;

struct SNP *snp_create(char *name, int32_t type, int32_t id);

SNP_RET_TYPE snp_exec(struct SNP *handle);

struct SNP_NODE_LIST *snp_get_nodes(struct SNP *handle);

void snp_set_sem_if(struct SNP *snp, SNP_SEM_WAIT wait, SNP_SEM_WAKE wake, void *sem_handle);

/**< 协议栈通用配置接口 */
SNP_RET_TYPE snp_set_log_if(SNP_LOG_IF log_if);

void snp_print_all(struct SNP *handle);

#ifdef __cplusplus
}
#endif

#endif // __LIBSNP_INCLUDE_SNP_H__

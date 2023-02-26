#ifndef __LIBSNP_SRC_INCLUDE_SNP_STD_PROCESS_H__
#define __LIBSNP_SRC_INCLUDE_SNP_STD_PROCESS_H__

#include "snp_node_internal.h"


#ifdef __cplusplus
extern "C" {
#endif

int32_t snp_link_dev_discovery(struct SNP_LINK *link);

int32_t snp_dev_network_sync(struct SNP_NODE *node);

int32_t snp_std_process_setup(struct SNP_NODE *node);


#ifdef __cplusplus
}
#endif

#endif // __LIBSNP_SRC_INCLUDE_SNP_STD_PROCESS_H__

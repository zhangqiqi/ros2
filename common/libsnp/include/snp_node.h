#ifndef __LIBSNP_INCLUDE_SNP_NODE_H__
#define __LIBSNP_INCLUDE_SNP_NODE_H__


#include "snp_defs.h"


#ifdef __cplusplus
extern "C" {
#endif

struct SNP_NODE;

typedef int32_t (*SNP_NODE_READ)(void *handle, uint8_t *buffer, int32_t size);
typedef int32_t (*SNP_NODE_WRITE)(void *handle, uint8_t *buffer, int32_t size);

struct SNP_NODE *snp_node_create();

#ifdef __cplusplus
}
#endif

#endif // __LIBSNP_INCLUDE_SNP_NODE_H__

#ifndef __LIBSNP_SRC_INCLUDE_SNP_BUFFER_H__
#define __LIBSNP_SRC_INCLUDE_SNP_BUFFER_H__

#include "snp_defs.h"

#ifdef __cplusplus
extern "C" {
#endif


struct SNP_BUFFER;

struct SNP_BUFFER *snp_buffer_create(int32_t size);

#ifdef __cplusplus
}
#endif

#endif // __LIBSNP_SRC_INCLUDE_SNP_BUFFER_H__

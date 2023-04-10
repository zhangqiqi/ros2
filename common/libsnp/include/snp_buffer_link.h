#ifndef __LIBSNP_INCLUDE_SNP_BUFFER_LINK_H__
#define __LIBSNP_INCLUDE_SNP_BUFFER_LINK_H__

#include "snp_defs.h"
#include "snp_buffer.h"


#ifdef __cplusplus
extern "C" {
#endif


struct SNP_BUFFER_LINK;

struct SNP_BUFFER_LINK *snp_buffer_link_create(int32_t buffer_size);

void snp_buffer_link_destory(struct SNP_BUFFER_LINK *buffer_link);

/**< 连接的一端使用的读写接口 */
int32_t snp_buffer_link_src_read(void *handle, struct SNP_BUFFER *buffer);

int32_t snp_buffer_link_src_write(void *handle, struct SNP_BUFFER *buffer);

/**< 连接的另一端使用的读写接口 */
int32_t snp_buffer_link_dst_read(void *handle, struct SNP_BUFFER *buffer);

int32_t snp_buffer_link_dst_write(void *handle, struct SNP_BUFFER *buffer);

#ifdef __cplusplus
}
#endif

#endif // __LIBSNP_INCLUDE_SNP_BUFFER_LINK_H__

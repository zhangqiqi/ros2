#ifndef __LIBSNP_SRC_INCLUDE_SNP_BUFFER_H__
#define __LIBSNP_SRC_INCLUDE_SNP_BUFFER_H__

#include "snp_defs.h"

#ifdef __cplusplus
extern "C" {
#endif


struct SNP_BUFFER;

struct SNP_BUFFER *snp_buffer_create(int32_t size);

void snp_buffer_destory(struct SNP_BUFFER *buffer);

SNP_RET_TYPE snp_buffer_setup_locker(struct SNP_BUFFER *buffer, void *lock_handle, SNP_LOCK lock, SNP_UNLOCK unlock);

int32_t snp_buffer_write(struct SNP_BUFFER *buffer, uint8_t *data, int32_t len);

int32_t snp_buffer_read(struct SNP_BUFFER *buffer, uint8_t *data, int32_t len);

int32_t snp_buffer_read_from(struct SNP_BUFFER *dst_buffer, struct SNP_BUFFER *src_buffer);

int32_t snp_buffer_copyout(struct SNP_BUFFER *buffer, uint8_t *data, int32_t len);

int32_t snp_buffer_copyout_ptr(struct SNP_BUFFER *buffer, uint8_t **data, int32_t len);

int32_t snp_buffer_copyout_from(struct SNP_BUFFER *dst_buffer, struct SNP_BUFFER *src_buffer);

int32_t snp_buffer_drain(struct SNP_BUFFER *buffer, int32_t len);


#ifdef __cplusplus
}
#endif

#endif // __LIBSNP_SRC_INCLUDE_SNP_BUFFER_H__

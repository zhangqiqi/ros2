#ifndef __LIBSNP_SRC_INCLUDE_SNP_BUFFER_H__
#define __LIBSNP_SRC_INCLUDE_SNP_BUFFER_H__

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif


struct SSNP_BUFFER;

struct SSNP_BUFFER *ssnp_buffer_create(int32_t size);

void ssnp_buffer_destory(struct SSNP_BUFFER *buffer);

int32_t ssnp_buffer_write(struct SSNP_BUFFER *buffer, uint8_t *data, int32_t len);

int32_t ssnp_buffer_read(struct SSNP_BUFFER *buffer, uint8_t *data, int32_t len);

int32_t ssnp_buffer_read_from(struct SSNP_BUFFER *dst_buffer, struct SSNP_BUFFER *src_buffer);

int32_t ssnp_buffer_copyout(struct SSNP_BUFFER *buffer, uint8_t *data, int32_t len);

int32_t ssnp_buffer_copyout_ptr(struct SSNP_BUFFER *buffer, uint8_t **data, int32_t len);

int32_t ssnp_buffer_copyout_from(struct SSNP_BUFFER *dst_buffer, struct SSNP_BUFFER *src_buffer);

int32_t ssnp_buffer_drain(struct SSNP_BUFFER *buffer, int32_t len);


#ifdef __cplusplus
}
#endif

#endif // __LIBSNP_SRC_INCLUDE_SNP_BUFFER_H__

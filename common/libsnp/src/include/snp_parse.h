#ifndef __LIBSNP_SRC_INCLUDE_SNP_PARSE_H__
#define __LIBSNP_SRC_INCLUDE_SNP_PARSE_H__


#include "snp_defs.h"
#include "snp_buffer.h"
#include "snp_msgs.h"

#ifdef __cplusplus
extern "C" {
#endif

uint32_t snp_proto_unpack(struct SNP_BUFFER *buffer, struct SNP_FRAME **frame);

uint32_t snp_proto_pack(struct SNP_BUFFER *buffer, struct SNP_FRAME *frame, uint8_t *msg, int32_t len);

#ifdef __cplusplus
}
#endif

#endif // __LIBSNP_SRC_INCLUDE_SNP_PARSE_H__

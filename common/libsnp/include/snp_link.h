#ifndef __LIBSNP_INCLUDE_SNP_LINK_H__
#define __LIBSNP_INCLUDE_SNP_LINK_H__

#include "snp_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 连接对象类型
 */
enum SNP_LINK_TYPE {
	SLT_UNKNOWN,      /**< 未知类型 */
	SLT_PHYSICAL_LINK,      /**< 物理连接 */
	SLT_SOFTWARE_LINK,      /**< 软件连接 */
	SLT_VIRTUAL_LINK,      /**< 虚拟连接 */
};


typedef int32_t (*SNP_LINK_READ)(void *handle, struct SNP_BUFFER *buffer);
typedef int32_t (*SNP_LINK_WRITE)(void *handle, struct SNP_BUFFER *buffer);

struct SNP_LINK *snp_link_create(struct SNP_NODE *src, struct SNP_NODE *dst, enum SNP_LINK_TYPE type);

void snp_link_destory(struct SNP_LINK *link);

int32_t snp_link_setup_rw_cb(struct SNP_LINK *link, SNP_LINK_READ read, SNP_LINK_WRITE write, void *handle);




#ifdef __cplusplus
}
#endif

#endif // __LIBSNP_INCLUDE_SNP_LINK_H__

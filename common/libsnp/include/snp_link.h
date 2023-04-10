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


/**
 * @brief 读取数据到指定buffer
 * @param handle 读操作句柄
 * @param buffer 写入buffer
 * @return 0 成功 其它 失败
 */
typedef int32_t (*SNP_LINK_READ)(void *handle, struct SNP_BUFFER *buffer);

/**
 * @brief 将buffer数据发送出去
 * @param handle 写操作句柄
 * @param buffer 读出buffer
 * @return 0 成功 其它 失败
 */
typedef int32_t (*SNP_LINK_WRITE)(void *handle, struct SNP_BUFFER *buffer);

struct SNP_LINK *snp_link_create(struct SNP_NODE *src, struct SNP_NODE *dst, enum SNP_LINK_TYPE type);

void snp_link_destory(struct SNP_LINK *link);

void snp_link_exec(struct SNP_LINK *link);

int32_t snp_link_setup_rw_cb(struct SNP_LINK *link, SNP_LINK_READ read, SNP_LINK_WRITE write, void *handle);

int32_t snp_link_forward_msg(struct SNP_LINK *link, struct SNP_FRAME *frame);

int32_t snp_link_send_broadcast_msg(struct SNP_LINK *link, int32_t msg_type, void *msg, int32_t size);

int32_t snp_link_send_single_msg(struct SNP_LINK *link, int32_t msg_type, void *msg, int32_t size);

int32_t snp_link_send_direct_msg(struct SNP_LINK *link, int32_t msg_type, void *msg, int32_t size);

#ifdef __cplusplus
}
#endif

#endif // __LIBSNP_INCLUDE_SNP_LINK_H__

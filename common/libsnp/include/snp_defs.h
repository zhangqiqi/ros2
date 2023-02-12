#ifndef __LIBSNP_INCLUDE_SNP_DEFS_H__
#define __LIBSNP_INCLUDE_SNP_DEFS_H__

#include <stdint.h>
#include <stddef.h>
#include <memory.h>
#include <malloc.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SNP_DEBUG(fmt, ...) printf(fmt)
#define SNP_NOTICE(fmt, ...) printf(fmt)
#define SNP_ERROR(fmt, ...) printf(fmt)


typedef int32_t SNP_RET_TYPE;
#define SNP_RET_OK (0)      /**< 返回成功 */
#define SNP_RET_NULLPTR_ERR (-1)      /**< 空指针错误 */

/**
 * @brief 调试信息输出枚举
 */
enum SNP_LOG_TYPE {
	SLT_DEBUG,
	SLT_NOTICE,
	SLT_ERROR
};

typedef void (*SNP_LOG_IF)(enum SNP_LOG_TYPE type, char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif // __LIBSNP_INCLUDE_SNP_DEFS_H__

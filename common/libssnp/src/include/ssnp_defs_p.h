#ifndef __LIBSSNP_SRC_INCLUDE_SSNP_DEFS_P_H__
#define __LIBSSNP_SRC_INCLUDE_SSNP_DEFS_P_H__

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "ssnp.h"

#ifdef __cplusplus
extern "C" {
#endif


#define SSNP_LOG(fmt...) \
do\
{\
	if (NULL == log_ctrl.log_print)\
	{\
		break;\
	}\
	log_ctrl.log_print(log_ctrl.log_handle, fmt);\
} while (false)


#define SSNP_DEBUG(fmt...)\
do\
{\
	if (log_ctrl.print_type <= SLT_DEBUG)\
	{\
		SSNP_LOG(fmt);\
	}\
} while (false)


#define SSNP_INFO(fmt...) \
do\
{\
	if (log_ctrl.print_type <= SLT_INFO)\
	{\
		SSNP_LOG(fmt);\
	}\
} while (false)


#define SSNP_ERROR(fmt...)\
do\
{\
	if (log_ctrl.print_type <= SLT_ERROR)\
	{\
		SSNP_LOG(fmt);\
	}\
} while (false)




enum SSNP_LOG_TYPE {
	SLT_DEBUG,
	SLT_INFO,
	SLT_ERROR
};

struct SSNP_LOG_CTRL {
	enum SSNP_LOG_TYPE print_type;      /**< 当前输出日志等级 */

	void *log_handle;      /**< 日志输出操作句柄 */
	SSNP_LOG_PRINT_CB log_print;      /**< 日志输出接口 */
};


extern struct SSNP_LOG_CTRL log_ctrl;


#ifdef __cplusplus
}
#endif

#endif //  __LIBSSNP_SRC_INCLUDE_SSNP_DEFS_P_H__


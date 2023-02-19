#ifndef __LIBSNP_SRC_INCLUDE_SNP_DEFS_P_H__
#define __LIBSNP_SRC_INCLUDE_SNP_DEFS_P_H__

#include "snp_defs.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief 系统锁资源相关创建代码
 */
#define SNP_LOCKER_CREATE() \
void * locker_handle;\
SNP_LOCK lock;\
SNP_UNLOCK unlock


/**
 * @brief snp锁 加锁函数
 */
#define SNP_LOCK(OBJ) \
do\
{\
	if ((NULL != OBJ->lock) && (NULL != OBJ->unlock))\
	{\
		OBJ->lock(OBJ->locker_handle);\
	}\
} while (false)


/**
 * @brief snp锁 解锁函数
 */
#define SNP_UNLOCK(OBJ) \
do\
{\
	if ((NULL != OBJ->lock) && (NULL != OBJ->unlock))\
	{\
		OBJ->unlock(OBJ->locker_handle);\
	}\
} while (false)


/**
 * @brief 系统信号量资源创建代码
 */
#define SNP_SEM_CREATE() \
void *sem_handle;\
SNP_SEM_WAIT wait;\
SNP_SEM_WAKE wake


/**
 * @brief 系统信号量等待接口
 */
#define SNP_WAIT(OBJ, ret) \
do\
{\
	if ((NULL != OBJ->wait) && (NULL != OBJ->wake))\
	{\
		ret = OBJ->wait(OBJ->sem_handle);\
	}\
} while (false)


/**
 * @brief 系统信号量唤醒接口
 */
#define SNP_WAKE(OBJ, ret) \
do\
{\
	if ((NULL != OBJ->wait) && (NULL != OBJ->wake))\
	{\
		ret = OBJ->wake(OBJ->sem_handle);\
	}\
} while (false)



#ifdef __cplusplus
}
#endif

#endif // __LIBSNP_SRC_INCLUDE_SNP_DEFS_P_H__

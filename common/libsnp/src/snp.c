#include "snp.h"
#include "snp_node.h"
#include "snp_defs_p.h"
#include "snp_std_process.h"

SNP_LOG_IF snp_log_print = NULL;      /**< 协议栈日志输出接口 */

/**
 * @brief 协议栈管理结构
 */
struct SNP {
	SNP_LOCKER_CREATE();
	SNP_SEM_CREATE();

	struct SNP_NODE_LIST *nodes;
};


/**
 * @brief 协议栈调试信息输出接口
 * @param snp 待设置接口的协议栈对象
 * @param log_if 待设置的信息输出接口
 * @return SNP_RET_OK 成功 其他 失败
 */
SNP_RET_TYPE snp_set_log_if(SNP_LOG_IF log_if)
{
	if (NULL == log_if)
	{
		return SNP_RET_NULLPTR_ERR;
	}

	snp_log_print = log_if;

	SNP_NOTICE("snp setup log if %p success\r\n", log_if);

	return SNP_RET_OK;
}


/**
 * @brief 协议栈信号量设置接口
 * @param[in] snp 协议栈对象
 * @param[in] wait 等待接口
 * @param[in] wake 唤醒接口
 * @param[in] sem_handle 接口应用句柄
 */
void snp_set_sem_if(struct SNP *snp, SNP_SEM_WAIT wait, SNP_SEM_WAKE wake, void *sem_handle)
{
	snp->sem_handle = sem_handle;
	snp->wait = wait;
	snp->wake = wake;
}


/**
 * @brief 协议栈启动执行函数 需要放到单独函数中执行
 * @param[in] handle 待执行的协议栈对象
 * @return SNP_RET_OK 成功 其他 失败
*/
SNP_RET_TYPE snp_exec(struct SNP *handle)
{
	SNP_RET_TYPE ret_code = SNP_RET_OK;
	int32_t sem_ret = 0;

	if (NULL == handle)
	{
		return SNP_RET_NULLPTR_ERR;
	}

	do
	{
		SNP_WAIT(handle, sem_ret);
		if (0 == sem_ret)
		{
			/**< 信号量唤醒 */
		}
		else
		{
			/**< 信号量超时 */
		}

		SNP_LOCK(handle);
		snp_node_exec(handle->nodes);
		SNP_UNLOCK(handle);

		snp_nodes_print_all(handle->nodes);
	} while (true);

	return ret_code;
}


/**
 * @brief 创建新的协议栈节点
 * @param[in] name 协议栈主节点名
 * @param[in] type 协议栈主节点设备类型
 * @param[in] id 协议栈主节点设备id
 * @return SNP_RET_TYPE 
 */
struct SNP *snp_create(char *name, int32_t type, int32_t id)
{
	struct SNP *_new_snp = (struct SNP *)malloc(sizeof(struct SNP));

	if (NULL == _new_snp)
	{
		return NULL;
	}

	memset(_new_snp, 0, sizeof(struct SNP));
	_new_snp->nodes = snp_node_list_create();
	if (NULL == _new_snp->nodes)
	{
		free(_new_snp);
		return NULL;
	}

	struct SNP_NODE *_main_node = snp_node_create(_new_snp->nodes, name, type, id);
	if (NULL == _main_node)
	{
		free(_new_snp->nodes);
		free(_new_snp);
		return NULL;
	}

	snp_std_process_setup(_main_node);

	return _new_snp;
}


/**
 * @brief 获取目标协议栈的节点集合
 * @param handle 目标协议栈对象指针
 * @return struct SNP_NODE_LIST* 获取到的协议栈节点集合指针
 */
struct SNP_NODE_LIST *snp_get_nodes(struct SNP *handle)
{
	if (NULL == handle)
	{
		return NULL;
	}

	return handle->nodes;
}


/**
 * @brief 打印协议栈的全部信息，包括构造信息，统计信息等
 * @param handle 
 */
void snp_print_all(struct SNP *handle)
{
	snp_nodes_print_all(handle->nodes);
}

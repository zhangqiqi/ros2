#include "snp.h"
#include "snp_node.h"
#include "snp_defs.h"
#include "snp_defs_p.h"
#include "snp_std_process.h"

// #include <memory.h>
// #include <malloc.h>

#include <stdio.h>
#include <stdarg.h>

static void snp_default_log_if(enum SNP_LOG_TYPE type, char *fmt, ...);


SNP_LOG_IF snp_log_print = snp_default_log_if;      /**< 协议栈日志输出接口 */

SNP_MALLOC snp_malloc = malloc;      /**< 动态内存分配接口重载指针 */
SNP_FREE snp_free = free;      /**< 动态内存释放接口重载指针 */

/**
 * @brief 协议栈管理结构
 */
struct SNP {
	SNP_LOCKER_CREATE();

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
 * @brief 协议栈默认日志输出接口
 * @param type 日志等级
 * @param fmt 格式化字符串
 * @param ... 参数列表
 */
static void snp_default_log_if(enum SNP_LOG_TYPE type, char *fmt, ...)
{
	static const char *type_str[3] = {
		"D",
		"N",
		"E"
	};

	static uint16_t cnt = 0;
	char prefix[128] = {0};

	snprintf(prefix, sizeof(prefix) - 1, "[%5d][%s]%s", cnt++, type_str[type], fmt);

	va_list args;
	va_start(args, fmt);
	vprintf(prefix, args);
	va_end(args);
}


/**
 * @brief 设置协议栈动态内存分配相关接口
 * @param malloc_if 动态内存申请接口
 * @param free_if 动态内存释放接口
 * @return SNP_RET_TYPE SNP_RET_OK 成功 其它 失败
 */
SNP_RET_TYPE snp_set_mem_if(SNP_MALLOC malloc_if, SNP_FREE free_if)
{
	if (NULL == malloc_if || NULL == free_if)
	{
		return SNP_RET_NULLPTR_ERR;
	}

	snp_malloc = malloc_if;
	snp_free = free_if;

	return SNP_RET_OK;
}


/**
 * @brief 协议栈启动执行函数 需要应用层定时调用，并传递两次调用间隔时间
 * @param handle 待执行的协议栈对象
 * @param elapsed_ms 距上次调用执行的时间 ms
 * @return SNP_RET_OK 成功 其他 失败
*/
SNP_RET_TYPE snp_exec(struct SNP *handle, int32_t elapsed_ms)
{
	if (NULL == handle)
	{
		return SNP_RET_NULLPTR_ERR;
	}

	SNP_LOCK(handle);
	snp_node_exec(handle->nodes);
	SNP_UNLOCK(handle);

	snp_nodes_print_all(handle->nodes);

	return SNP_RET_OK;
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
	struct SNP *_new_snp = (struct SNP *)snp_malloc(sizeof(struct SNP));

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
 * @brief 为协议栈创建新的物理节点，该节点将直接挂在协议栈上，并与根节点建立物理关联
 * @param handle 协议栈管理句柄
 * @param read 与该节点连接的数据读取接口
 * @param write 与该节点连接的数据写入接口
 * @param rw_handle 读写接口操作句柄
 * @return 创建完成的新节点对象指针
 */
struct SNP_NODE *snp_create_physical_node(struct SNP *handle, SNP_LINK_READ read, SNP_LINK_WRITE write, void *rw_handle)
{
	if (NULL == handle)
	{
		return NULL;
	}

	SNP_LOCK(handle);

	struct SNP_NODE *unknown_node = snp_node_create(handle->nodes, "unknown_dev", SDT_UNKNOWN_DEV, -1);
	do
	{
		if (NULL == unknown_node)
		{
			SNP_DEBUG("%s %d: create new node failed\r\n", __func__, __LINE__);
			break;
		}

		struct SNP_LINK *link = snp_link_create(snp_node_get_root(handle->nodes), unknown_node, SLT_PHYSICAL_LINK);
		if (NULL == link)
		{
			SNP_DEBUG("%s %d: create new link failed, destory node %p\r\n", __func__, __LINE__, unknown_node);
			snp_node_destory(unknown_node);
			unknown_node = NULL;
			break;
		}

		snp_link_setup_rw_cb(link, read, write, rw_handle);
	} while (false);

	SNP_UNLOCK(handle);

	return unknown_node;
}


/**
 * @brief 向协议栈的指定节点发送消息
 * @param handle 协议栈对象
 * @param dst_node 目标节点对象
 * @param msg_type 消息类型
 * @param msg 消息体
 * @param size 消息体字节数
 * @return 发送成功的字节数
 */
int32_t snp_send_msg_by_node(struct SNP *handle, struct SNP_NODE *dst_node, int32_t msg_type, void *msg, int32_t size)
{
	int32_t ret = 0;
	struct SNP_NODE *local_node = snp_get_local_node(handle);

	struct SNP_LINK *_dst_link = snp_link_get_by_node(local_node, dst_node);
	
	return snp_link_write(_dst_link, msg_type, msg, size);
}


/**
 * @brief 向协议栈的指定节点发送消息
 * @param handle 协议栈对象
 * @param name 目标节点名
 * @param msg_type 消息类型
 * @param msg 消息体
 * @param size 消息体字节数
 * @return 发送成功的字节数
 */
int32_t snp_send_msg_by_name(struct SNP *handle, char *name, int32_t msg_type, void *msg, int32_t size)
{
	int32_t ret = 0;

	return ret;
}


/**
 * @brief 向协议栈的指定节点发送消息
 * @param handle 协议栈对象
 * @param id 目标节点id
 * @param msg_type 消息类型
 * @param msg 消息体
 * @param size 消息体字节数
 * @return 发送成功的字节数
 */
int32_t snp_send_msg_by_id(struct SNP *handle, int32_t id, int32_t msg_type, void *msg, int32_t size)
{
	int32_t ret = 0;

	return ret;
}


/**
 * @brief 获取协议栈本地节点
 * @param handle 协议栈管理对象
 * @return 得到的本地节点对象
 */
struct SNP_NODE *snp_get_local_node(struct SNP *handle)
{
	struct SNP_NODE *_node = NULL;

	SNP_LOCK(handle);
	_node = snp_node_get_root(handle->nodes);
	SNP_UNLOCK(handle);

	return _node;
}


/**
 * @brief 打印协议栈的全部信息，包括构造信息，统计信息等
 * @param handle 
 */
void snp_print_all(struct SNP *handle)
{
	SNP_LOCK(handle);
	snp_nodes_print_all(handle->nodes);
	SNP_UNLOCK(handle);
}

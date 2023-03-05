#include "snp.h"
#include "snp_node.h"
#include "snp_defs.h"
#include "snp_defs_p.h"
#include "snp_std_process.h"

#include <stdio.h>
#include <stdarg.h>
#include <sys/time.h>
#include <time.h>

static enum SNP_LOG_TYPE snp_default_log_level = SLT_NOTICE;
static void snp_default_log_if(enum SNP_LOG_TYPE type, char *fmt, ...);

SNP_LOG_IF snp_log_print = snp_default_log_if;      /**< 协议栈日志输出接口 */

SNP_MALLOC snp_malloc = malloc;      /**< 动态内存分配接口重载指针 */
SNP_FREE snp_free = free;      /**< 动态内存释放接口重载指针 */


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
	char prefix[256] = {0};

	if (type < snp_default_log_level)
	{
		return;
	}

	struct timeval tv = {0};
	gettimeofday(&tv, NULL);

	time_t timer;
	timer = time(NULL);
	struct tm tm = {0};
	localtime_r(&timer, &tm);

	snprintf(prefix, sizeof(prefix) - 1, "[%04d-%02d-%02d %02d:%02d:%02d.%6ld][%5d][%s]%s", 
		(1900 + tm.tm_year), (1 + tm.tm_mon), tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, tv.tv_usec,
		cnt++, type_str[type], fmt
	);

	va_list args;
	va_start(args, fmt);
	vprintf(prefix, args);
	va_end(args);
}


/**
 * @brief 设置是否启用日志转发服务，启用日志转发服务，会将日志信息，自动转发给日志服务器
 * @param handle 日志转发服务使用的日志服务器所属协议栈
 * @param en true 使能 false 去使能
 */
void snp_set_log_forward_enable(struct SNP *handle, bool en)
{

}


/**
 * @brief 设置协议栈的日志输出级别，仅使用默认日志输出接口时有效
 * @param level 日志输出的最低级别
 */
void snp_set_log_level(enum SNP_LOG_TYPE level)
{
	snp_default_log_level = level;
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
 * @brief 协议栈设备网络同步任务
 * @param handle 协议栈对象
 * @param elapsed_ms 距上次调用的间隔时间 ms
 */
static void snp_dev_network_sync_task(struct SNP *handle, int32_t elapsed_ms)
{
	handle->snp_network_sync_tick -= elapsed_ms;
	if (handle->snp_network_sync_tick > 0)
	{
		return;
	}

	/**< 协议栈网络同步倒计时结束，进行一次本地网络结构信息广播，并重置计时器的值 */
	SNP_DEBUG("snp %p start dev network sync exec\r\n", handle);
	handle->snp_network_sync_tick = SNP_NETWORK_SYNC_TICK_CNT;
	snp_dev_network_sync(snp_node_get_root(handle->nodes));
}


/**
 * @brief 协议栈定时任务
 * @param handle 协议栈对象
 * @param elapsed_ms 距上次调用的间隔时间 ms
 */
static void snp_timer_task(struct SNP *handle, int32_t elapsed_ms)
{

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
	handle->snp_tick += elapsed_ms;
	SNP_DEBUG("snp %p exec start, snp tick: %d\r\n", handle, handle->snp_tick);

	snp_dev_network_sync_task(handle, elapsed_ms);
	snp_timer_task(handle, elapsed_ms);

	snp_node_exec(handle->nodes);
	SNP_UNLOCK(handle);

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
	_new_snp->snp_network_sync_tick = SNP_NETWORK_SYNC_TICK_CNT;
	_new_snp->nodes = snp_node_list_create();
	if (NULL == _new_snp->nodes)
	{
		free(_new_snp);
		return NULL;
	}

	struct SNP_NODE *_main_node = snp_node_create(_new_snp, _new_snp->nodes, name, type, id);
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
 * @brief 为协议栈创建与根节点相关联的，这将创建一个未知节点，以及建立该未知节点与根节点的关联
 * @param handle 协议栈对象
 * @param type 该节点与根节点的关系
 * @return 未知节点和根节点的连接对象 NULL 创建失败
 */
static struct SNP_LINK *snp_create_root_linked_node(struct SNP *handle, enum SNP_LINK_TYPE type)
{
	struct SNP_LINK *link = NULL;
	struct SNP_NODE *unknown_node = snp_node_create(handle, handle->nodes, "unknown_dev", SDT_UNKNOWN_DEV, -1);

	do
	{
		if (NULL == unknown_node)
		{
			SNP_DEBUG("%s %d: create new node failed\r\n", __func__, __LINE__);
			break;
		}

		link = snp_link_create(snp_node_get_root(handle->nodes), unknown_node, type);
		if (NULL == link)
		{
			SNP_DEBUG("%s %d: create new link failed, destory node %p\r\n", __func__, __LINE__, unknown_node);
			snp_node_destory(unknown_node);
			break;
		}
	} while (false);
	
	return link;
}


/**
 * @brief 为协议栈创建新的物理节点，该节点将直接挂在协议栈上，并与根节点建立物理关联
 * @param handle 协议栈管理句柄
 * @param read 与该节点连接的数据读取接口
 * @param write 与该节点连接的数据写入接口
 * @param rw_handle 读写接口操作句柄
 * @return 创建完成的新节点对象指针
 */
struct SNP_LINK *snp_create_physical_node(struct SNP *handle, SNP_LINK_READ read, SNP_LINK_WRITE write, void *rw_handle)
{
	if (NULL == handle)
	{
		return NULL;
	}

	SNP_LOCK(handle);

	struct SNP_LINK *link = snp_create_root_linked_node(handle, SLT_PHYSICAL_LINK);
	if (NULL != link)
	{
		snp_link_setup_rw_cb(link, read, write, rw_handle);
	}

	SNP_UNLOCK(handle);

	return link;
}


/**
 * @brief 创建新的协议栈软件节点，该节点将直接挂在协议站上，并与根节点建立软件关联
 * @param handle 协议栈管理句柄
 * @param read 与该节点连接的数据读取接口
 * @param write 与该节点连接的数据写入接口
 * @param rw_handle 读写接口操作句柄
 * @return 创建完成的新节点对象指针
 */
struct SNP_LINK *snp_create_software_node(struct SNP *handle, SNP_LINK_READ read, SNP_LINK_WRITE write, void *rw_handle)
{
	if (NULL == handle)
	{
		return NULL;
	}

	SNP_LOCK(handle);

	struct SNP_LINK *link = snp_create_root_linked_node(handle, SLT_SOFTWARE_LINK);
	if (NULL != link)
	{
		snp_link_setup_rw_cb(link, read, write, rw_handle);
	}

	SNP_UNLOCK(handle);

	return link;
}


/**
 * @brief 创建新的协议栈虚拟节点，该节点将直接挂在协议站上，并与根节点建立虚拟关联
 * @param handle 协议栈管理句柄
 * @param read 与该节点连接的数据读取接口
 * @param write 与该节点连接的数据写入接口
 * @param rw_handle 读写接口操作句柄
 * @return 创建完成的新节点对象指针
 */
struct SNP_LINK *snp_create_virtual_node(struct SNP *handle, SNP_LINK_READ read, SNP_LINK_WRITE write, void *rw_handle)
{
	if (NULL == handle)
	{
		return NULL;
	}

	SNP_LOCK(handle);

	struct SNP_LINK *link = snp_create_root_linked_node(handle, SLT_VIRTUAL_LINK);
	if (NULL != link)
	{
		snp_link_setup_rw_cb(link, read, write, rw_handle);
	}

	SNP_UNLOCK(handle);

	return link;
}


/**
 * @brief 向协议栈的指定节点发送消息
 * @param handle 协议栈对象
 * @param dst_node 目标节点对象
 * @param msg_type 消息类型
 * @param msg 消息体
 * @param size 消息体字节数
 * @return SNP_RET_OK 成功 其它 失败
 */
int32_t snp_send_msg_by_node(struct SNP *handle, struct SNP_NODE *dst_node, int32_t msg_type, void *msg, int32_t size)
{
	int32_t ret = 0;
	struct SNP_NODE *local_node = snp_get_local_node(handle);

	struct SNP_LINK *_dst_link = snp_link_get_by_node(local_node, dst_node);
	
	return snp_link_write(_dst_link, msg_type, msg, size);
}


/**
 * @brief 从协议栈广播一条消息
 * @param handle 目标协议栈对象
 * @param msg_type 消息类型
 * @param msg 消息体
 * @param size 消息体长度
 * @return int32_t SNP_RET_OK 成功 其它 失败
 */
int32_t snp_broadcast_msg(struct SNP *handle, int32_t msg_type, void *msg, int32_t size)
{
	if (NULL == handle)
	{
		SNP_ERROR("%s %d: handle ptr is %p\r\n", __func__, __LINE__, handle);
		return SNP_RET_NULLPTR_ERR;
	}

	SNP_LOCK(handle);
	struct SNP_NODE *root = snp_node_get_root(handle->nodes);
	SNP_UNLOCK(handle);

	return snp_node_broadcast_msg(root, msg_type, msg, size);
}


/**
 * @brief 获取系统中的全部节点信息
 * @param handle 协议栈类型
 * @param nodes_info 节点信息写入数组
 * @param cnt 期望获取的最多节点个数
 * @return 实际获取的节点信息个数
 */
int32_t snp_get_nodes_info(struct SNP *handle, struct SNP_NODE_INFO *nodes_info, int32_t cnt)
{
	struct SNP_NODE *_var_node = NULL;

	int32_t ret_cnt = 0;
	TAILQ_FOREACH(_var_node, handle->nodes, NODE)
	{
		if (ret_cnt < cnt)
		{
			strncpy(nodes_info[ret_cnt].name, _var_node->name, sizeof(nodes_info[ret_cnt].name) - 1);
			nodes_info[ret_cnt].id = _var_node->id;
			nodes_info[ret_cnt].type = _var_node->type;
			ret_cnt++;
			continue;
		}
		break;
	}

	return ret_cnt;
}


/**
 * @brief 向协议栈的指定节点发送消息
 * @param handle 协议栈对象
 * @param name 目标节点名
 * @param msg_type 消息类型
 * @param msg 消息体
 * @param size 消息体字节数
 * @return SNP_RET_OK 成功 其它 失败
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
 * @return SNP_RET_OK 成功 其它 失败
 */
int32_t snp_send_msg_by_id(struct SNP *handle, int32_t id, int32_t msg_type, void *msg, int32_t size)
{
	struct SNP_LINK *_link = snp_link_get_by_id(snp_node_get_root(handle->nodes), id);
	if (NULL == _link)
	{
		SNP_DEBUG("send msg(%d) to id(%d) failed, dir not exist\r\n", msg_type, id);
		return SNP_RET_DIR_NOT_EXIST;
	}
	return snp_link_send_direct_msg(_link, msg_type, msg, size);
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

#include "libsnp_app.h"

#include "snp.h"
#include "snp_node.h"
#include "snp_buffer.h"
#include "snp_msgs.h"
#include "snp_parse.h"
#include "snp_node_internal.h"

#include "defs.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include <stdio.h>
#include <stdarg.h>

static struct SNP *snp_handle = NULL;

const char *type_str[3] = {
	"D",
	"N",
	"E"
};

/**
 * @brief snp 服务器日志输出接口
 * @param type 日志消息等级
 * @param fmt 日志消息
 * @param ... 参数列表
 */
static void snp_log_printf(enum SNP_LOG_TYPE type, char *fmt, ...)
{
	char prefix[128] = {0};
	char print_str[512] = {0};
	static uint16_t cnt = 0;

	snprintf(prefix, sizeof(prefix) - 1, "[%5d][%s]%s", cnt++, type_str[type], fmt);

	va_list args;
	va_start(args, fmt);
	// vprintf(prefix, args);
	vsnprintf(print_str, sizeof(print_str) - 1, prefix, args);
	va_end(args);

	HAL_UART_Transmit(&huart2, (uint8_t *)print_str, strlen(print_str) + 1, 0xffff);
}


/**
 * @brief 协议栈等待接口
 * @param handle 应用句柄
 * @return int32_t 0 主动唤醒 1 超时唤醒
 */
static int32_t snp_wait_cb(void *handle)
{
	static int32_t wait_cnt = 0;
	int32_t ret = 0;

	ret = osSemaphoreWait(handle, 1000);
	snp_log_printf(SLT_DEBUG, "\r\nsnp wake cnt: %d\r\n", ++wait_cnt);

	return ret;
}


/**
 * @brief 协议栈唤醒接口
 * @param handle 应用句柄
 * @return int32_t 0 唤醒成功 -1 失败
 */
static int32_t snp_wake_cb(void *handle)
{
	int32_t ret = 0;

	return ret;
}


void libsnp_task_thread_exec(void const *argument)
{
	snp_print_all(snp_handle);

	snp_exec(snp_handle);
}


/**
 * @brief snp协议栈任务初始化
 */
void libsnp_app_init(void)
{
	/**< 初始化协议栈库功能接口 */
	snp_set_log_if(snp_log_printf);

	/**< 初始化协议栈初始表结构 */
	snp_handle = snp_create("bau_monitor", SDK_BAU_MONITOR, 1);

	struct SNP_NODE_LIST *nodes = snp_get_nodes(snp_handle);
	struct SNP_NODE *root_node = snp_node_get_root(nodes);

	struct SNP_NODE *unknown_node1 = snp_node_create(nodes, "unknown_dev", SDT_UNKNOWN_DEV, -1);
	struct SNP_NODE *unknown_node2 = snp_node_create(nodes, "unknown_dev", SDT_UNKNOWN_DEV, -1);
	
	snp_link_setup_rw_cb(
		snp_link_create(root_node, unknown_node1, SLT_PHYSICAL_LINK),
		NULL,
		NULL,
		NULL
	);

	snp_link_setup_rw_cb(
		snp_link_create(root_node, unknown_node2, SLT_PHYSICAL_LINK),
		NULL,
		NULL,
		NULL
	);

	/**< 初始化协议栈对象功能接口 */
	osSemaphoreDef(sem);
	snp_set_sem_if(snp_handle, snp_wait_cb, snp_wake_cb, osSemaphoreCreate(osSemaphore(sem), 1));

	// osThreadDef(libsnp_task_thread, libsnp_task_thread_exec, osPriorityBelowNormal, 0, 10240);
	// osThreadCreate(osThread(libsnp_task_thread), NULL);

	libsnp_task_thread_exec(NULL);
}

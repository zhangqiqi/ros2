#include "snp.h"
#include "snp_node.h"

SNP_LOG_IF snp_log_print = NULL;      /**< 协议栈日志输出接口 */

/**
 * @brief 协议栈管理结构
 */
struct SNP {
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
 * @brief 协议栈启动执行函数 需要放到单独函数中执行
 * @param[in] handle 待执行的协议栈对象
 * @return SNP_RET_OK 成功 其他 失败
*/
SNP_RET_TYPE snp_exec(struct SNP *handle)
{
	SNP_RET_TYPE ret_code = SNP_RET_OK;


	return ret_code;
}


/**
 * @brief 创建新的协议栈节点
 * @return SNP_RET_TYPE 
 */
struct SNP *snp_create(void)
{
	struct SNP *_new_snp = (struct SNP *)malloc(sizeof(struct SNP));

	_new_snp->nodes = snp_node_list_create();

	struct SNP_NODE *_main_node = snp_node_create(_new_snp->nodes);

	struct SNP_NODE *_sub_test_node = snp_node_create(_new_snp->nodes);

	struct SNP_LINK *_test_link = snp_link_create(_main_node, _sub_test_node);

	return _new_snp;
}


/**
 * @brief 打印协议栈的全部信息，包括构造信息，统计信息等
 * @param handle 
 */
void snp_print_all(struct SNP *handle)
{
	snp_nodes_print_all(handle->nodes);
}

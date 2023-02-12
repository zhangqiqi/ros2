#include "snp.h"



/**
 * @brief 协议栈管理结构
 */
struct SNP {
	SNP_LOG_IF log_print;      /**< 协议栈日志输出接口 */
};


/**
 * @brief 协议栈调试信息输出接口
 * @param snp 待设置接口的协议栈对象
 * @param log_if 待设置的信息输出接口
 * @return SNP_RET_OK 成功 其他 失败
 */
SNP_RET_TYPE snp_set_log_if(struct SNP *snp, SNP_LOG_IF log_if)
{
	if ((NULL == snp) || (NULL == log_if))
	{
		return SNP_RET_NULLPTR_ERR;
	}

	snp->log_print = log_if;
	snp->log_print(SLT_NOTICE, "snp(%p) setup log if %p success\r\n", snp, log_if);

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

	return _new_snp;
}

#include "ssnp.h"
#include "ssnp_defs_p.h"
#include "ssnp_shell.h"
#include "ssnp_defs_p.h"

void *ssnp_log_handle;      /**< 日志输出操作句柄 */	
SSNP_LOG_PRINT_CB ssnp_log_print = NULL;      /**< 日志输出接口 */


/**
 * @brief 协议栈日志输出管理对象
 */
struct SSNP_LOG_CTRL log_ctrl = {
	.print_type = SLT_INFO,
	.log_handle = NULL,
	.log_print = NULL
};


/**
 * @brief ssnp对象
 */
struct SSNP {

	void *recv_handle;      /**< 接受数据操作句柄 */
	SSNP_RECV_CB recv_cb;      /**< 接收数据操作回调 */
	struct SSNP_BUFFER *recv_buf;      /**< 接收数据缓存区 */
	struct SSNP_MSGS_PUB_LIST *recv_msg_pub;      /**< 接收消息发布管理对象 */

	void *trans_handle;      /**< 发送数据操作句柄 */
	SSNP_TRANS_CB trans_cb;      /**< 发送数据操作回调 */
	struct SSNP_BUFFER *trans_buf;      /**< 发送数据缓存区 */
};


int32_t ssnp_log_print_setup(void *log_handle, SSNP_LOG_PRINT_CB log_cb)
{
	if (NULL == log_cb)
	{
		return -1;
	}
	
	log_ctrl.log_handle = log_handle;
	log_ctrl.log_print = log_cb;	
	return 0;
}


struct SSNP *ssnp_create()
{
	struct SSNP *_new_ssnp = (struct SSNP *)malloc(sizeof(struct SSNP));

	if (NULL != _new_ssnp)
	{
		memset(_new_ssnp, 0, sizeof(struct SSNP));
		SSNP_INFO("create new ssnp (%p) success", _new_ssnp);
	}

	return _new_ssnp;
}


/**
 * @brief 接收设备shell指令
 * @param cb_handle 接收响应注册使用的应用句柄
 * @param link 接收到消息的连接对象
 * @param msg 接收到的消息体
 * @return 0 成功 其它 失败
 */
static int32_t ssnp_shell_req_msg_proc(void *cb_handle, struct SSNP *ssnp, struct SSNP_FRAME *msg)
{
	struct SMT_SHELL_REQ_MSG *_msg = (struct SMT_SHELL_REQ_MSG *)msg->payload;

	char res_str[256] = {0};
	struct SMT_SHELL_RES_MSG *res_msg = (struct SMT_SHELL_RES_MSG *)res_str;

	ssnp_shell_exec(_msg->req_str, res_msg->res_str, sizeof(res_str) - sizeof(struct SMT_SHELL_RES_MSG) - 1);

	return 0;
}


/**
 * @brief 设置协议栈数据接收接口
 * @param ssnp 目标协议栈对象
 * @param recv_handle 数据接收操作句柄
 * @param recv_cb 数据接受操作回调
 * @return int32_t 0 设置成功 其它 失败
 */
int32_t ssnp_recv_if_setup(struct SSNP *ssnp, void *recv_handle, SSNP_RECV_CB recv_cb)
{
	if (NULL == ssnp || NULL == recv_cb)
	{
		return -1;
	}

	ssnp->recv_handle = recv_handle;
	ssnp->recv_cb = recv_cb;
	ssnp->recv_buf = ssnp_buffer_create(SSNP_RECV_BUFFER_SIZE);
	ssnp->recv_msg_pub = ssnp_msgs_create_pub_list();

	ssnp_msgs_add_pub_cb(ssnp->recv_msg_pub, SMT_SHELL_REQ, ssnp_shell_req_msg_proc, NULL);

	return 0;
}


/**
 * @brief 设置协议栈数据发送接口
 * @param ssnp 目标协议栈对象
 * @param trans_handle 数据发送操作句柄
 * @param trans_cb 数据发送操作回调
 * @return int32_t 0 设置成功 其它 失败
 */
int32_t ssnp_trans_if_setup(struct SSNP *ssnp, void *trans_handle, SSNP_TRANS_CB trans_cb)
{
	if (NULL == ssnp || NULL == trans_cb)
	{
		return -1;
	}

	ssnp->trans_handle = trans_handle;
	ssnp->trans_cb = trans_cb;
	ssnp->trans_buf = ssnp_buffer_create(SSNP_TRANS_BUFFER_SIZE);
	return 0;
}


/**
 * @brief 设置消息监听
 * @param ssnp 目标协议栈栈对象
 * @param type 目标消息类型
 * @param msg_cb 监听回调
 * @param cb_handle 监听回调用户句柄
 * @return 0 设置成功 其它 设置失败
 */
int32_t ssnp_msgs_listener_setup(struct SSNP *ssnp, int32_t type, SSNP_MSG_CB msg_cb, void *cb_handle)
{
	if ((NULL == ssnp) || (NULL == msg_cb))
	{
		return -1;
	}

	return ssnp_msgs_add_pub_cb(ssnp->recv_msg_pub, type, msg_cb, cb_handle);
}


/**
 * @brief 消息发送接口
 * @param ssnp 协议栈对象
 * @param type 消息类型
 * @param msg 消息数据
 * @param len 消息数据长度
 * @return 0 成功 其它 失败
 */
int32_t ssnp_send_msg(struct SSNP *ssnp, int32_t type, uint8_t *msg, int32_t len)
{
	if (NULL == ssnp)
	{
		return -1;
	}

	struct SSNP_FRAME _frame = {
		.frame_type = type,
	};

	ssnp_msgs_pack(ssnp->trans_buf, &_frame, msg, len);
	return 0;
}


/**
 * @brief 协议栈执行数据接收流程
 * 
 * @param ssnp 目标协议栈对象
 * @return int32_t 接收到的数据长度 小于0 接收异常
 */
static int32_t ssnp_recv_data(struct SSNP *ssnp)
{
	int32_t ret = 0;

	do
	{
		if (NULL == ssnp->recv_cb)
		{
			break;
		}

		if (NULL == ssnp->recv_buf)
		{
			ret = -1;
			break;
		}

		ret = ssnp->recv_cb(ssnp->recv_handle, ssnp->recv_buf);
	} while (false);
	
	return ret;
}


/**
 * @brief 协议栈发送数据流程
 * 
 * @param ssnp 目标协议栈对象
 * @return int32_t 发送的数据长度 小于0 发送异常
 */
static int32_t ssnp_trans_data(struct SSNP *ssnp)
{
	int32_t ret = 0;

	do
	{
		if (NULL == ssnp->trans_cb)
		{
			break;
		}

		if (NULL == ssnp->trans_buf)
		{
			ret = -1;
			break;
		}

		ret = ssnp->trans_cb(ssnp->trans_handle, ssnp->trans_buf);
	} while (false);
	
	return ret;
}


/**
 * @brief 处理协议栈接收到的数据
 * 
 * @param ssnp 目标协议栈对象
 * @return int32_t 0 处理完成 其它 异常
 */
static int32_t ssnp_proc_recv_data(struct SSNP *ssnp)
{
	int32_t ret = 0;
	struct SSNP_FRAME *_frame = NULL;

	do
	{
		ret = ssnp_msgs_unpack(ssnp->recv_buf, &_frame);
		SSNP_DEBUG("unpack data len: %d", ret);
		if (NULL == _frame)
		{
			if (ret > 0)
			{
				ssnp_buffer_drain(ssnp->recv_buf, ret);
			}
			break;
		}

		/**< 检测到新帧 处理该帧 */
		ssnp_msgs_pub(ssnp->recv_msg_pub, ssnp, _frame);
		if (ret > 0)
		{
			ssnp_buffer_drain(ssnp->recv_buf, ret);
		}

		ret = 0;
		_frame = NULL;
	} while (true);

	return 0;
}


/**
 * @brief 执行ssnp协议栈
 * 
 * @param ssnp 目标ssnp对象
 * @return int32_t 执行结果
 */
int32_t ssnp_exec(struct SSNP *ssnp)
{
	if (NULL == ssnp)
	{
		return -1;
	}

   	ssnp_recv_data(ssnp);
	ssnp_proc_recv_data(ssnp);
	ssnp_trans_data(ssnp);
	
	return 0;
}

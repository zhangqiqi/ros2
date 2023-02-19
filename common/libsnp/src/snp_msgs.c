#include "snp_msgs.h"
#include "queue.h"


/**
 * @brief snp消息发布管理结构体
 */
struct SNP_MSGS_PUB {
	int32_t type;      /**< 发布消息类型 */

	void *cb_handle;      /**< 监听发布消息的回调句柄 */
	SNP_MSG_CB cb;      /**< 消息发布回调 */

	TAILQ_ENTRY(SNP_MSGS_PUB) MSGS;
};

TAILQ_HEAD(SNP_MSGS_PUB_LIST, SNP_MSGS_PUB);


/**
 * @brief 创建消息发布队列
 * @return 创建完成的发布队列 NULL 创建失败
 */
struct SNP_MSGS_PUB_LIST *snp_msgs_create_pub_list(void)
{
	struct SNP_MSGS_PUB_LIST *_ret_pub_list = (struct SNP_MSGS_PUB_LIST *)malloc(sizeof(struct SNP_MSGS_PUB_LIST));

	if (NULL != _ret_pub_list)
	{
		TAILQ_INIT(_ret_pub_list);
	}

	return _ret_pub_list;
}


/**
 * @brief 向队列添加新的消息监听回调
 * @param[in] pub_list 目标发布队列
 * @param[in] type 期望监听的消息类型
 * @param[in] msb_cb 消息监听回调函数
 * @param[in] cb_handle 监听回调函数的应用句柄
 * @return SNP_RET_OK 成功 其它 失败
 */
SNP_RET_TYPE snp_msgs_add_pub_cb(struct SNP_MSGS_PUB_LIST *pub_list, int32_t type, SNP_MSG_CB msg_cb, void *cb_handle)
{
	if (NULL == pub_list)
	{
		return SNP_RET_NULLPTR_ERR;
	}

	struct SNP_MSGS_PUB *_new_msg_pub = (struct SNP_MSGS_PUB *)malloc(sizeof(struct SNP_MSGS_PUB));
	if (NULL == _new_msg_pub)
	{
		return SNP_RET_NO_MEM;
	}

	_new_msg_pub->type = type;
	_new_msg_pub->cb = msg_cb;
	_new_msg_pub->cb_handle = cb_handle;

	TAILQ_INSERT_TAIL(pub_list, _new_msg_pub, MSGS);

	return SNP_RET_OK;
}

/**
 * @brief 向消息发布链表发布新的消息包
 * @parma[in] pub_list 目标消息链表
 * @param[in] link 消息所属的连接
 * @param[in] msg 消息包对象
 * @return SNP_RET_OK 成功 其它 失败
 */
SNP_RET_TYPE snp_msgs_pub(struct SNP_MSGS_PUB_LIST *pub_list, struct SNP_LINK *link, struct SNP_FRAME *msg)
{
	if ((NULL == pub_list) || (NULL == link) || (NULL == msg))
	{
		return SNP_RET_NULLPTR_ERR;
	}

	struct SNP_MSGS_PUB *_var = NULL;

	TAILQ_FOREACH(_var, pub_list, MSGS)
	{
		if (_var->type != msg->frame_type)
		{
			continue;
		}

		if (NULL != _var->cb)
		{
			_var->cb(_var->cb_handle, link, msg);
		}
	}

	return SNP_RET_OK;
}
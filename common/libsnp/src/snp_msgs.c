#include "snp_msgs.h"
#include "snp_defs_p.h"
#include "snp_buffer.h"

#define SNP_MSG_MAGIC (0x5AA55AA5)      /**< 默认栈帧标识符 */


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
 * @brief 从目标缓存区中解析获取第一条完整消息
 * @param buffer 待解析处理的缓存区数据
 * @param frame 解析得到的第一条有效帧数据地址
 * @return uint32_t 本次处理后缓存区失效数据长度
 */
uint32_t snp_msgs_unpack(struct SNP_BUFFER *buffer, struct SNP_FRAME **frame)
{
	uint32_t drain_size = 0;
	int32_t read_size = 0;
	struct SNP_FRAME *_frame = NULL;

	do
	{
		read_size = snp_buffer_copyout_ptr(buffer, (uint8_t **)&_frame, sizeof(struct SNP_FRAME));
		if ((NULL == _frame) || (read_size < sizeof(struct SNP_FRAME)))
		{
			/**< 数据长度不够 */
			SNP_DEBUG("%s %d: _frame is %p, buffer read size (%d), sizeof(struct SNP_FRAME) (%d)\r\n",
				__func__, __LINE__, _frame, read_size, sizeof(struct SNP_FRAME)
			);
			break;
		}

		if (SNP_MSG_MAGIC != _frame->magic)
		{
			/**< magic无法识别 移除一个无效字符 */
			SNP_DEBUG("find snp frame head magic %x failed, cur magic is %x, remove unvalid char\r\n", SNP_MSG_MAGIC, _frame->magic);
			snp_buffer_drain(buffer, 1);
			continue;
		}

		if ((sizeof(struct SNP_FRAME) + _frame->frame_len) > SNP_DEFAULT_BUFFER_SIZE)
		{
			/**< 帧数据长度错误，无法接收 */
			SNP_DEBUG("snp frame len error, frame len(%d + %d) > max len(%d)\r\n", 
				SNP_MSG_MAGIC, _frame->magic, 
				sizeof(struct SNP_FRAME), _frame->frame_len, SNP_DEFAULT_BUFFER_SIZE
			);
			snp_buffer_drain(buffer, 1);
			continue;
		}

		read_size = snp_buffer_copyout_ptr(buffer, (uint8_t **)&_frame, sizeof(struct SNP_FRAME) + _frame->frame_len);

		if (read_size < (sizeof(_frame) + _frame->frame_len))
		{
			/**< 完整的snp包还未接收完整 */
			SNP_DEBUG("package read size %d < request size %d, waitfor...\r\n", read_size, sizeof(_frame) + _frame->frame_len);
			break;
		}
		
		drain_size = read_size;
		*frame = _frame;
		break;
	} while (true);

	return drain_size;
}


/**
 * @brief 构造新的协议栈帧并写入到目标缓存区中
 * @param[in] buffer 待写入的缓存区对象
 * @param[in] frame 协议帧头信息
 * @param[in] msg 协议帧负载数据
 * @param[in] len 协议栈负载长度
 * @return 本次写入到缓存区中的有效数据长度
 */
uint32_t snp_msgs_pack(struct SNP_BUFFER *buffer, struct SNP_FRAME *frame, uint8_t *msg, int32_t len)
{
	uint32_t write_size = 0;

	if (NULL == buffer || NULL == frame)
	{
		return 0;
	}

	frame->magic = SNP_MSG_MAGIC;
	frame->frame_len = len;
	frame->crc32 = 0;      /**< 测试阶段先不用 */

	write_size = snp_buffer_write(buffer, (uint8_t *)frame, sizeof(struct SNP_FRAME));
	write_size += snp_buffer_write(buffer, msg, len);

	return write_size;
}



/**
 * @brief 创建消息发布队列
 * @return 创建完成的发布队列 NULL 创建失败
 */
struct SNP_MSGS_PUB_LIST *snp_msgs_create_pub_list(void)
{
	struct SNP_MSGS_PUB_LIST *_ret_pub_list = (struct SNP_MSGS_PUB_LIST *)snp_malloc(sizeof(struct SNP_MSGS_PUB_LIST));

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

	struct SNP_MSGS_PUB *_new_msg_pub = (struct SNP_MSGS_PUB *)snp_malloc(sizeof(struct SNP_MSGS_PUB));
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
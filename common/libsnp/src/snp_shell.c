#include "snp_shell.h"
#include "snp_defs_p.h"

#include <string.h>


/**
 * @brief shell指令类型
 */
enum SNP_SHELL_CMD_TYPE {
	SSCT_UNKNOWN,      /**< 未知/未使用 的指令 */
	SSCT_NORMAL,      /**< 普通指令 需要进行字符串完全匹配 */
	SSCT_PLACEHOLDER,      /**< 占位符指令 只要存在即可 不需要识别具体的内容 */
};

/**
 * @brief 同级指令列表
 */
SIMPLEQ_HEAD(SNP_SHELL_LIST, SNP_SHELL_CMD);

/**
 * @brief shell指令基本节点类型
 */
struct SNP_SHELL_CMD {
	char cmd[16];      /**< 单指令字符串 */
	enum SNP_SHELL_CMD_TYPE type;      /**< 单指令类型 */
	SNP_SHELL_PROC_CB proc_cb;      /**< 指令处理回调 */

	struct SNP_SHELL_LIST sub_cmds;      /**< 下级指令列表 */

	SIMPLEQ_ENTRY(SNP_SHELL_CMD) SHELL;
};

static struct SNP_SHELL_LIST __snp_shell = {0};
static struct SNP_SHELL_LIST *snp_shell = &__snp_shell;      /**< shell根节点 */

static struct SNP_SHELL_CMD shell_cmd_init_arr[32] = {0};      /**< 预分配32个节点使用 */

/**
 * @brief 初始化协议栈shell模块
 */
void snp_shell_init(void)
{
	SIMPLEQ_INIT(snp_shell);
}


/**
 * @brief 获取新的指令节点
 * @return 获取到的指令节点指针 NULL 获取失败
 */
static struct SNP_SHELL_CMD *__snp_shell_get_new_cmd_node(void)
{
	int num = sizeof(shell_cmd_init_arr) / sizeof(shell_cmd_init_arr[0]);
	int i = 0;

	for (i = 0; i < num; i++)
	{
		if (SSCT_UNKNOWN == shell_cmd_init_arr[i].type)
		{
			return &shell_cmd_init_arr[i];
		}
	}

	return NULL;
}


/**
 * @brief 字符串指令分割提取，默认使用空格来分割
 * @param cmd 指令字符串
 * @param len 字符串长度
 * @param cmd_list 分割得到的字符串地址数组
 * @param num 分割得到的字符串地址数组最大个数
 * @return 得到的字符串个数
 */
static int32_t __snp_shell_cmd_split(char *cmd, char **cmd_list, int32_t num)
{
	char *save_ptr = NULL;
	int i = 0;

	if (NULL == (cmd_list[i] = strtok_r(cmd, " ", &save_ptr)))
	{
		return 0;
	}
	i++;

	do
	{
		if (NULL == (cmd_list[i] = strtok_r(NULL, " ", &save_ptr)))
		{
			break;
		}
		i++;

		if (i >= num)
		{
			break;
		}
	} while (1);

	return i;
}


/**
 * @brief 安装指令处理回调
 * @param descriptor 指令描述信息
 * @param num 指令描述信息个数
 * @param proc_cb 指令处理回调函数
 * @return 0 安装成功 其它 失败
 */
int32_t snp_shell_setup(char **descriptor, int32_t num, SNP_SHELL_PROC_CB proc_cb)
{
	struct SNP_SHELL_LIST *_var_cmd_list = snp_shell;
	struct SNP_SHELL_CMD *_var_cmd;
	int i = 0;
	for (i = 0; i < num; i++)
	{
		SIMPLEQ_FOREACH(_var_cmd, _var_cmd_list, SHELL)
		{
			if (0 == strcmp(descriptor[i], _var_cmd->cmd))
			{
				break;
			}
		}

		if (NULL == _var_cmd)
		{
			/**< 当前层级没有该指令，需要增加新指令支持 */
			_var_cmd = __snp_shell_get_new_cmd_node();
			if (NULL == _var_cmd)
			{
				/**< 分配不出新节点了，直接返回失败 */
				return -1;
			}

			if (0 == strcmp(descriptor[i], SNP_SHELL_PLACEHOLDER))
			{
				/**< 是个占位符节点 */
				_var_cmd->type = SSCT_PLACEHOLDER;
			}
			else
			{
				/**< 普通节点 */
				_var_cmd->type = SSCT_NORMAL;
			}

			strncpy(_var_cmd->cmd, descriptor[i], sizeof(_var_cmd->cmd) - 1);
			SIMPLEQ_INIT(&_var_cmd->sub_cmds);
			SIMPLEQ_INSERT_TAIL(_var_cmd_list, _var_cmd, SHELL);
		}
		else
		{
			/**< 当前层级有了对应的指令了，看一下下一层指令有没有 */
		}

		_var_cmd_list = &_var_cmd->sub_cmds;
	}

	_var_cmd->proc_cb = proc_cb;

	return 0;
}


/**
 * @brief 执行shell指令
 * @param cmd_list shell 指令字符串数组
 * @param num shell指令字符串个数
 * @param res_str 指令处理回复字符串写入地址
 * @param res_size 指令响应最大回复字符串长度
 * @return 实际响应的字符串长度 0 无回复
 */
int32_t snp_shell_exec(char *cmd_str, char *res_str, int32_t res_size)
{
	struct SNP_SHELL_LIST *_var_cmd_list = snp_shell;
	struct SNP_SHELL_CMD *_var_cmd = NULL;
	struct SNP_SHELL_CMD *_var_placeholder_cmd = NULL;

	char *cmd_list[16] = {0};
	int32_t num = __snp_shell_cmd_split(cmd_str, cmd_list, sizeof(cmd_list) / sizeof(cmd_list[0]));

	int i = 0;
	for (i = 0; i < num; i++)
	{
		/**< 先找完全匹配的指令 */
		SIMPLEQ_FOREACH(_var_cmd, _var_cmd_list, SHELL)
		{
			if (0 == strcmp(_var_cmd->cmd, cmd_list[i]))
			{
				break;
			}
			if (SSCT_PLACEHOLDER == _var_cmd->type)
			{
				/**< 记录下找到的通配符指令，以便后续使用 */
				_var_placeholder_cmd = _var_cmd;
			}
		}

		if (NULL != _var_cmd)
		{
			_var_cmd_list = &_var_cmd->sub_cmds;
		}
		else if (NULL != _var_placeholder_cmd)
		{
			_var_cmd_list = &_var_placeholder_cmd->sub_cmds;
		}
		else
		{
			/**< 找不到能用的节点，那就直接结束了，返回失败 */
			return 0;
		}
	}

	if (NULL != _var_cmd)
	{
		return _var_cmd->proc_cb(cmd_list, num, res_str, res_size);
	}

	if (NULL != _var_placeholder_cmd)
	{
		return _var_placeholder_cmd->proc_cb(cmd_list, num, res_str, res_size);
	}

	return 0;
}

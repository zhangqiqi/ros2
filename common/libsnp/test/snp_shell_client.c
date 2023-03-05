#include "snp.h"
#include "snp_node.h"
#include "snp_buffer.h"
#include "snp_msgs.h"
#include "snp_node_internal.h"
#include "snp_buffer_link.h"

#include "libevent_tcp_link.h"

#include <stdio.h>
#include <stdarg.h>
#include <pthread.h>


static void *snp_shell_client_task(void *arg)
{
	libevent_tcp_link_exec(((void **)arg)[0], ((void **)arg)[1]);
}


/**
 * @brief 创建一个shell客户端
 */
int32_t snp_shell_tcp_client_create(char *name, int32_t id, char *ip, uint16_t port)
{
	snp_set_log_level(SLT_ERROR);

	struct SNP *snp_shell_client = snp_create(name, SDT_SHELL_SERVER, id);
	void *tcp_handle = libevent_base_create();

	libevent_tcp_client_link_create(tcp_handle, snp_shell_client, ip, port);

	void *arg[2] = {tcp_handle, snp_shell_client};

	pthread_t ntid;
	pthread_create(&ntid, NULL, snp_shell_client_task, arg);

	char shell_str[256] = {0};

	struct SSM_SHELL_REQ_MSG *_shell_msg = (struct SSM_SHELL_REQ_MSG *)shell_str;
	while (true)
	{
		memset(_shell_msg->req_str, 0, sizeof(shell_str));
		printf("snp shell >>");
		scanf("%s", _shell_msg->req_str);
		_shell_msg->req_len = strlen(_shell_msg->req_str) + 1;
		snp_broadcast_msg(snp_shell_client, SSM_SHELL_REQ, _shell_msg, sizeof(struct SSM_SHELL_REQ_MSG) + _shell_msg->req_len);
	}
}

#include "snp.h"
#include "snp_node.h"
#include "snp_buffer.h"
#include "snp_msgs.h"
#include "snp_node_internal.h"
#include "snp_buffer_link.h"
#include "snp_shell.h"

#include "libevent_tcp_link.h"

#include <stdio.h>
#include <stdarg.h>


int32_t test_snp_link_read(void *handle, struct SNP_BUFFER *buffer)
{
	struct SNP_FRAME frame = {0};
	struct SSM_DISCOVERY_RES_MSG msg = {0};

	struct SNP_NODE *_node = (struct SNP_NODE *)handle;

	frame.src_node_id = _node->id;
	frame.dst_node_id = SNP_BROADCAST_ID;
	frame.frame_type = SSM_DISCOVERY_RES;
	frame.frame_seq = _node->seq++;

	strncpy(msg.name, _node->name, sizeof(msg.name));
	msg.type = _node->type;
	msg.id = _node->id;

	int pack_size = snp_msgs_pack(buffer, &frame, (uint8_t *)&msg, sizeof(msg));
	printf("read buffer size: %d\r\n", pack_size);

	return 0;
}


int32_t test_snp_link_write(void *handle, struct SNP_BUFFER *buffer)
{
	uint8_t data[1024] = {0};

	int32_t len = snp_buffer_read(buffer, data, sizeof(data));

	struct SNP_NODE *_node = (struct SNP_NODE *)handle;

	struct SNP_FRAME *frame = (struct SNP_FRAME *)data;

	printf("link handle %p send data: dst node %d, frame type %d, msg len %d\r\n", 
		_node, frame->dst_node_id, frame->frame_type, frame->frame_len
	);

	return len;
}


static void snp_dev_tcp_server_link_test(uint16_t port)
{
	struct SNP *snp_relay_server = snp_create("relay_server", SDT_RELAY_SERVER, 1);

	void *tcp_handle = libevent_base_create();

	// libevent_tcp_client_link_create(tcp_handle, snp_relay_server, "10.114.80.121", 60000);

	libevent_tcp_server_link_create(tcp_handle, snp_relay_server, port);

	libevent_tcp_link_exec(tcp_handle, snp_relay_server);
}


/**
 * @brief tcp客户端连接测试
 */
static void snp_dev_tcp_client_link_test(uint16_t port)
{
	time_t timer;
	timer = time(NULL);

	struct SNP *snp_relay_server = snp_create("relay_server", SDT_RELAY_SERVER, timer % 10000);

	void *tcp_handle = libevent_base_create();

	libevent_tcp_client_link_create(tcp_handle, snp_relay_server, "10.1.65.194", port);

	libevent_tcp_link_exec(tcp_handle, snp_relay_server);
}


/**
 * @brief 多设备组网测试
 * @param cnt 执行循环次数
 */
static void snp_dev_network_sync_test(int32_t cnt)
{
	/**< 协议栈 1 构造 */
	struct SNP *snp_relay_server_1 = snp_create("relay_server1", SDT_RELAY_SERVER, 1);
	snp_create_software_node(snp_relay_server_1, NULL, NULL, NULL);
	snp_create_software_node(snp_relay_server_1, NULL, NULL, NULL);


	/**< 协议栈 2 构造 */
	struct SNP *snp_relay_server_2 = snp_create("relay_server2", SDT_RELAY_SERVER, 2);
	snp_create_software_node(snp_relay_server_2, NULL, NULL, NULL);
	snp_create_software_node(snp_relay_server_2, NULL, NULL, NULL);
	snp_create_software_node(snp_relay_server_2, NULL, NULL, NULL);

	/**< 协议栈 3 构造 */
	struct SNP *snp_relay_server_3 = snp_create("relay_server3", SDT_RELAY_SERVER, 3);
	snp_create_software_node(snp_relay_server_3, NULL, NULL, NULL);
	snp_create_software_node(snp_relay_server_3, NULL, NULL, NULL);

	/**< 协议栈 1 和 2 建立关联 */
	struct SNP_BUFFER_LINK *_test_buffer_link_1_to_2 = snp_buffer_link_create(1024);
	snp_create_software_node(snp_relay_server_1, snp_buffer_link_src_read, snp_buffer_link_src_write, _test_buffer_link_1_to_2);
	snp_create_software_node(snp_relay_server_2, snp_buffer_link_dst_read, snp_buffer_link_dst_write, _test_buffer_link_1_to_2);

	/**< 协议栈2 和 3 建立关联 */
	struct SNP_BUFFER_LINK *_test_buffer_link_2_to_3 = snp_buffer_link_create(1024);
	snp_create_software_node(snp_relay_server_2, snp_buffer_link_src_read, snp_buffer_link_src_write, _test_buffer_link_2_to_3);
	snp_create_software_node(snp_relay_server_3, snp_buffer_link_dst_read, snp_buffer_link_dst_write, _test_buffer_link_2_to_3);

	int32_t cycle_cnt = 0;
	do
	{
		// sleep(1);
		printf("\r\nsnp wake up cnt: %d\r\n", ++cycle_cnt);
		printf("######################### snp server 1 #################################\r\n");
		snp_exec(snp_relay_server_1, 1 * 1000);
		snp_print_all(snp_relay_server_1);
		printf("########################################################################\r\n");

		printf("######################### snp server 2 #################################\r\n");
		snp_exec(snp_relay_server_2, 1 * 1000);
		snp_print_all(snp_relay_server_2);
		printf("########################################################################\r\n");

		printf("######################### snp server 3 #################################\r\n");
		snp_exec(snp_relay_server_3, 1 * 1000);
		snp_print_all(snp_relay_server_3);
		printf("########################################################################\r\n");
	} while (--cnt);
}


/**
 * @brief 缓存区连接测试
 * @param cnt 执行循环次数
 */
static void snp_buffer_link_test(int32_t cnt)
{
	struct SNP *snp_relay_server_1 = snp_create("relay_server1", SDT_RELAY_SERVER, 1);
	struct SNP *snp_relay_server_2 = snp_create("relay_server2", SDT_RELAY_SERVER, 2);

	struct SNP_BUFFER_LINK *_test_buffer_link = snp_buffer_link_create(1024);

	snp_create_software_node(snp_relay_server_1, snp_buffer_link_src_read, snp_buffer_link_src_write, _test_buffer_link);
	snp_create_software_node(snp_relay_server_2, snp_buffer_link_dst_read, snp_buffer_link_dst_write, _test_buffer_link);

	int32_t cycle_cnt = 0;
	do
	{
		sleep(1);
		printf("\r\nsnp wake up cnt: %d\r\n", ++cycle_cnt);
		printf("######################### snp server 1 #################################\r\n");
		snp_exec(snp_relay_server_1, 1 * 1000);
		snp_print_all(snp_relay_server_1);
		printf("########################################################################\r\n");

		printf("######################### snp server 2 #################################\r\n");
		snp_exec(snp_relay_server_2, 1 * 1000);
		snp_print_all(snp_relay_server_2);
		printf("########################################################################\r\n");
	} while (--cnt);
	
}


int32_t test_shell_cmd_proc(char **cmd_list, int32_t num, char *res_str, int32_t res_size)
{
	memset(res_str, 0, res_size);

	snprintf(res_str, res_size, "%d", 9999);

	return strlen(res_str);
}


static int32_t bau_shell_read_motor_speed(char **cmd_list, int32_t num, char *res_str, int32_t res_size)
{
	snprintf(res_str, res_size - 1, "996.icu\r\n");

	return strlen(res_str) + 1;
}


int main(int argc, char **argv)
{
	snp_set_log_level(SLT_NOTICE);

	if (argc < 2)
	{
		snp_dev_network_sync_test(7);
		return 0;
	}

	char *test_name = argv[1];

	if (0 == strcmp(test_name, "shell_tcp_client"))
	{
		snp_shell_tcp_client_create("shell_client", SDT_SHELL_SERVER, "10.1.65.194", 9999);
	}
	else if (0 == strcmp(test_name, "tcp_server"))
	{
		snp_shell_init();
		char *test_shell_cmd[] = {
			"read", "server_port"
		};
		snp_shell_setup(test_shell_cmd, 2, test_shell_cmd_proc);

		static const char *read_left_motor_speed[] = {"read", "left", "motor", "speed"};
		snp_shell_setup(read_left_motor_speed, 4, bau_shell_read_motor_speed);

		static const char *read_right_motor_speed[] = {"read", "right", "motor", "speed"};
		snp_shell_setup(read_right_motor_speed, 4, bau_shell_read_motor_speed);

		snp_dev_tcp_server_link_test(9999);
	}
	else if (0 == strcmp(test_name, "tcp_client"))
	{
		snp_dev_tcp_client_link_test(9999);
	}
	else
	{
	}

	return 0;
}


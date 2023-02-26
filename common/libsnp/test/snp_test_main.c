#include "snp.h"
#include "snp_node.h"
#include "snp_buffer.h"
#include "snp_msgs.h"
#include "snp_node_internal.h"
#include "snp_buffer_link.h"

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


/**
 * @brief 协议栈构造测试
 * @param cnt 执行循环次数
 */
static void snp_construct_test(int32_t cnt)
{
	struct SNP *snp_relay_server_handle = snp_create("relay_server", SDT_RELAY_SERVER, 1);

	struct SNP_NODE *bau_monitor = snp_node_create(NULL, "bau_monitor", SDK_BAU_MONITOR, 10);
	struct SNP_NODE *log_server = snp_node_create(NULL, "log_server", SDT_LOG_SERVER, 11);
	struct SNP_NODE *alarm_server = snp_node_create(NULL, "alarm_server", SDT_ALARM_SERVER, 12);
	struct SNP_NODE *timestamp_server = snp_node_create(NULL, "timestamp_server", SDT_TIMESTAMP_SERVER, 13);

	snp_create_physical_node(snp_relay_server_handle, test_snp_link_read, test_snp_link_write, bau_monitor);
	snp_create_physical_node(snp_relay_server_handle, test_snp_link_read, test_snp_link_write, log_server);
	snp_create_physical_node(snp_relay_server_handle, test_snp_link_read, test_snp_link_write, alarm_server);
	snp_create_physical_node(snp_relay_server_handle, test_snp_link_read, test_snp_link_write, timestamp_server);

	snp_print_all(snp_relay_server_handle);

	int32_t cycle_cnt = 0;
	do
	{
		// sleep(1);
		printf("\r\nsnp wake up cnt: %d\r\n", ++cycle_cnt);
		snp_exec(snp_relay_server_handle, 1 * 1000);
	} while (--cnt);
}


int main(int argc, char **argv)
{
	snp_set_log_level(SLT_DEBUG);

	snp_dev_network_sync_test(7);
	// snp_buffer_link_test(1000);

	// snp_construct_test(2);

	return 0;
}


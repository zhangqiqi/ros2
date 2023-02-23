#include "snp.h"
#include "snp_node.h"
#include "snp_buffer.h"
#include "snp_msgs.h"
#include "snp_node_internal.h"

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


int main(int argc, char **argv)
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

	int32_t cnt = 0;
	do
	{
		sleep(1);
		printf("\r\nsnp wake up cnt: %d\r\n", ++cnt);
		snp_exec(snp_relay_server_handle, 1 * 1000);
	} while (true);

	return 0;
}


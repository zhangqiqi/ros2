#include "snp.h"
#include "snp_node.h"
#include "snp_buffer.h"
#include "snp_msgs.h"
#include "snp_parse.h"
#include "snp_node_internal.h"

#include <stdio.h>
#include <stdarg.h>


const char *type_str[3] = {
	"D",
	"N",
	"E"
};

void test_snp_log_print(enum SNP_LOG_TYPE type, char *fmt, ...)
{
	char prefix[128] = {0};
	static uint16_t cnt = 0;

	snprintf(prefix, sizeof(prefix) - 1, "[%5d][%s]%s", cnt++, type_str[type], fmt);

	va_list args;
	va_start(args, fmt);
	vprintf(prefix, args);
	va_end(args);
}


int32_t test_snp_wait_cb(void *handle)
{
	static int32_t wait_cnt = 0;
	int32_t _usleep = *(int32_t *)handle;

	// usleep(_usleep); /**< usleep 在mingw32环境下不休眠，具体原因待排查 */
	sleep(1);
	printf("\r\nsnp wake cnt: %d\r\n", ++wait_cnt);

	return -1;
}

int32_t test_snp_wake_cb(void *handle)
{

}


int32_t test_snp_link_read(void *handle, struct SNP_BUFFER *buffer)
{
	struct SNP_FRAME frame = {0};
	struct SSM_DISCOVERY_RES_MSG msg = {0};

	struct SNP_NODE *_node = (struct SNP_NODE *)handle;

	frame.src_node_id = snp_node_get_id(_node);
	frame.dst_node_id = SNP_BOARDCAST_ID;
	frame.frame_type = SSM_DISCOVERY_RES;
	frame.frame_seq = _node->seq++;

	snp_node_get_name(_node, msg.name, sizeof(msg.name));
	msg.type = snp_node_get_type(_node);
	msg.id = snp_node_get_id(_node);

	int pack_size = snp_proto_pack(buffer, &frame, (uint8_t *)&msg, sizeof(msg));
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

	snp_set_log_if(test_snp_log_print);

	struct SNP_NODE_LIST *nodes = snp_get_nodes(snp_relay_server_handle);

	struct SNP_NODE *root_node = snp_node_get_root(nodes);

	struct SNP_NODE *unknown_node1 = snp_node_create(nodes, "unknown_dev", SDT_UNKNOWN_DEV, -1);
	struct SNP_NODE *unknown_node2 = snp_node_create(nodes, "unknown_dev", SDT_UNKNOWN_DEV, -1);
	struct SNP_NODE *unknown_node3 = snp_node_create(nodes, "unknown_dev", SDT_UNKNOWN_DEV, -1);
	struct SNP_NODE *unknown_node4 = snp_node_create(nodes, "unknown_dev", SDT_UNKNOWN_DEV, -1);


	struct SNP_NODE *bau_monitor = snp_node_create(NULL, "bau_monitor", SDK_BAU_MONITOR, 10);
	struct SNP_NODE *log_server = snp_node_create(NULL, "log_server", SDT_LOG_SERVER, 11);
	struct SNP_NODE *alarm_server = snp_node_create(NULL, "alarm_server", SDT_ALARM_SERVER, 12);
	struct SNP_NODE *timestamp_server = snp_node_create(NULL, "timestamp_server", SDT_TIMESTAMP_SERVER, 13);

	snp_link_setup_rw_cb(
		snp_link_create(root_node, unknown_node1, SLT_SOFTWARE_LINK),
		test_snp_link_read,
		test_snp_link_write,
		bau_monitor
	);

	snp_link_setup_rw_cb(
		snp_link_create(root_node, unknown_node2, SLT_SOFTWARE_LINK),
		test_snp_link_read,
		test_snp_link_write,
		log_server
	);

	snp_link_setup_rw_cb(
		snp_link_create(root_node, unknown_node3, SLT_SOFTWARE_LINK),
		test_snp_link_read,
		test_snp_link_write,
		alarm_server
	);

	snp_link_setup_rw_cb(
		snp_link_create(root_node, unknown_node4, SLT_SOFTWARE_LINK),
		test_snp_link_read,
		test_snp_link_write,
		timestamp_server
	);

	int32_t _sleep = 1000 * 1000;

	snp_set_sem_if(snp_relay_server_handle, test_snp_wait_cb, test_snp_wake_cb, &_sleep);

	snp_print_all(snp_relay_server_handle);

	snp_exec(snp_relay_server_handle);

	return 0;
}


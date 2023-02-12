#include "snp.h"
#include "snp_node.h"

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

int main(int argc, char **argv)
{
	struct SNP *snp_handle = snp_create();

	snp_set_log_if(test_snp_log_print);

	struct SNP_NODE_LIST *nodes = snp_get_nodes(snp_handle);

	struct SNP_NODE *root_node = snp_node_get_root(nodes);
	struct SNP_NODE *test_node1 = snp_node_create(nodes);
	struct SNP_NODE *test_node2 = snp_node_create(nodes);
	struct SNP_NODE *test_node3 = snp_node_create(nodes);
	struct SNP_NODE *test_node4 = snp_node_create(nodes);
	struct SNP_NODE *test_node5 = snp_node_create(nodes);
	struct SNP_NODE *test_node6 = snp_node_create(nodes);

	snp_link_create(root_node, test_node1);
	snp_link_create(root_node, test_node2);
	snp_link_create(root_node, test_node3);
	snp_link_create(root_node, test_node4);
	snp_link_create(root_node, test_node5);
	snp_link_create(root_node, test_node6);
	snp_link_create(test_node1, test_node2);
	snp_link_create(test_node1, test_node3);
	snp_link_create(test_node1, test_node4);
	snp_link_create(test_node1, test_node5);
	snp_link_create(test_node1, test_node6);
	snp_link_create(test_node2, test_node3);
	snp_link_create(test_node2, test_node1);
	snp_link_create(test_node3, test_node1);
	snp_link_create(test_node3, test_node2);
	snp_link_create(test_node3, test_node3);
	snp_link_create(test_node3, test_node4);
	snp_link_create(test_node3, test_node5);
	snp_link_create(test_node3, test_node6);
	snp_link_create(test_node4, test_node5);
	snp_link_create(test_node5, test_node6);


	snp_print_all(snp_handle);

	return 0;
}


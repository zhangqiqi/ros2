#include "snp.h"
#include "snp_node.h"

#include <stdio.h>
#include <stdarg.h>

void snp_log_print(enum SNP_LOG_TYPE type, char *fmt, ...)
{
	char prefix[128] = {0};

	snprintf(prefix, sizeof(prefix) - 1, "[%d]%s", type, fmt);

	va_list args;
	va_start(args, fmt);
	vprintf(prefix, args);
	va_end(args);
}

int main(int argc, char **argv)
{
	struct SNP *snp_handle = snp_create();
	struct SNP_NODE *snp_node = snp_node_create();

	printf("new snp handle: %p\r\n", snp_handle);

	printf("set snp(%p) log if(%p) ret: %x\r\n", snp_handle, snp_log_print, snp_set_log_if(snp_handle, snp_log_print));

	return 0;
}


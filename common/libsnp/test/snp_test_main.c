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

	printf("new snp handle: %p\r\n", snp_handle);
	printf("set snp log if(%p) ret: %x\r\n", test_snp_log_print, snp_set_log_if(test_snp_log_print));

	snp_print_all(snp_handle);

	return 0;
}


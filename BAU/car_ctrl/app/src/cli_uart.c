#include "cli_uart.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "defs.h"


char prtBuf[128];

#define LOG_UART_SEND(src, len)do\
							{\
								if(osKernelRunning() > 0)\
								{\
									HAL_UART_Transmit_DMA(&huart1, (uint8_t *)src, len);\
									osSemaphoreWait(semlog_tx_cpt, 20);\
								}\
								else\
								{\
									HAL_UART_Transmit(&huart1, (uint8_t *)src, len, 0xFFFF);\
								}\
							}\
							while(0)


static void send_log_info(char *pFormat, int new_line, va_list args)
{
	char *p = prtBuf;
	uint8_t len = 0;

	len += vsprintf(p + len, pFormat, args);
	if (new_line)
	{
		len += sprintf(p + len, "%s", "\r\n");
	}
	LOG_UART_SEND(p, len);
}

void log_print(char* fmt, ...)
{
	va_list args;
	osMutexWait(mutex_log,osWaitForever);
	va_start(args, fmt);
	send_log_info(fmt, 1, args);
	va_end(args);
	osMutexRelease(mutex_log);
}



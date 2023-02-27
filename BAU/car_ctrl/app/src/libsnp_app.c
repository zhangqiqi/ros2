#include "libsnp_app.h"

#include "snp.h"
#include "snp_node.h"
#include "snp_buffer.h"
#include "snp_msgs.h"
#include "snp_node_internal.h"

#include "defs.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include <stdio.h>
#include <stdarg.h>

static struct SNP *snp_handle = NULL;


#define HUART_DMA_BUFFER_SIZE (1024)

static uint8_t huart1_dma_buffer[HUART_DMA_BUFFER_SIZE] = {0};
static uint8_t huart2_dma_buffer[HUART_DMA_BUFFER_SIZE] = {0};


int32_t snp_uart_read(void *handle, struct SNP_BUFFER *buffer)
{
	UART_HandleTypeDef *huart = NULL;

	uint16_t buffer_size = 0;

	if ((UART_HandleTypeDef *)handle == &huart1)
	{
		HAL_UART_DMAStop(&huart1);
		buffer_size = huart1.RxXferSize - huart1.hdmarx->Instance->NDTR;
		snp_buffer_write(buffer, huart1_dma_buffer, buffer_size);
		HAL_UART_Receive_DMA(&huart1, huart1_dma_buffer, HUART_DMA_BUFFER_SIZE);
	}
	else if ((UART_HandleTypeDef *)handle == &huart2)
	{
		HAL_UART_DMAStop(&huart2);
		buffer_size = huart2.RxXferSize - huart2.hdmarx->Instance->NDTR;
		snp_buffer_write(buffer, huart2_dma_buffer, buffer_size);
		HAL_UART_Receive_DMA(&huart2, huart2_dma_buffer, HUART_DMA_BUFFER_SIZE);
	}
	else
	{

	}

	return 0;
}

int32_t snp_uart_write(void *handle, struct SNP_BUFFER *buffer)
{
	uint8_t *data = NULL;
	int32_t size = 1024;

	size = snp_buffer_copyout_ptr(buffer, &data, size);

	if (NULL != data)
	{
		HAL_UART_Transmit(handle, data, size, 100);
		snp_buffer_drain(buffer, size);
	}

	return 0;
}


void libsnp_task_thread_exec(void const *argument)
{
	snp_print_all(snp_handle);

	HAL_UART_Receive_DMA(&huart1, huart1_dma_buffer, HUART_DMA_BUFFER_SIZE);
	HAL_UART_Receive_DMA(&huart2, huart2_dma_buffer, HUART_DMA_BUFFER_SIZE);

	do
	{
		osDelay(10);
		snp_exec(snp_handle, 10);
	} while (true);
}



/**
 * @brief snp协议栈任务初始化
 */
void libsnp_app_init(void)
{
	/**< 初始化协议栈库功能接口 */
	snp_set_log_if(NULL);

	/**< 初始化协议栈初始表结构 */
	snp_handle = snp_create("bau_monitor", SDK_BAU_MONITOR, 1);

	snp_create_physical_node(snp_handle, snp_uart_read, snp_uart_write, &huart1);
	snp_create_physical_node(snp_handle, snp_uart_read, snp_uart_write, &huart2);

	osThreadDef(libsnp_task_thread, libsnp_task_thread_exec, osPriorityNormal, 0, 2048);
	osThreadCreate(osThread(libsnp_task_thread), NULL);
}

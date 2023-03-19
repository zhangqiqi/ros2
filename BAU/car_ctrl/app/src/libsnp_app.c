#include "libsnp_app.h"

#include "defs.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include <stdio.h>
#include <stdarg.h>

#include "snp.h"
#include "snp_node.h"
#include "snp_buffer.h"
#include "snp_msgs.h"
#include "snp_node_internal.h"
#include "snp_shell.h"

#include "motor.h"

extern struct MOTOR *left_motor;
extern struct MOTOR *right_motor;

struct DMA_BUFFER_CTRL {
	int32_t read_idx;

	uint8_t dma_buffer[SNP_DEFAULT_BUFFER_SIZE * 2];
};

static struct DMA_BUFFER_CTRL snp_rw_buffer = {0};

static int32_t bau_snp_link_read(void *handle, struct SNP_BUFFER *buffer)
{
	UART_HandleTypeDef *huart = (UART_HandleTypeDef *)handle;

	HAL_UART_DMAStop(huart);

	int32_t cur_read_cnt = huart->RxXferSize - huart->hdmarx->Instance->NDTR;

	if (cur_read_cnt >= sizeof(snp_rw_buffer.dma_buffer))
	{
		osDelay(1000);
	}
	else if (cur_read_cnt > 0)
	{
		if (snp_buffer_write(buffer, snp_rw_buffer.dma_buffer, cur_read_cnt) <= 0)
		{
			osDelay(1000);
		}
	}
	else
	{

	}

	HAL_UART_Receive_DMA(huart, snp_rw_buffer.dma_buffer, sizeof(snp_rw_buffer.dma_buffer));

	return 0;
}


static int32_t bau_snp_link_write(void *handle, struct SNP_BUFFER *buffer)
{
	UART_HandleTypeDef *huart = (UART_HandleTypeDef *)handle;

	uint8_t *data = NULL;

	int32_t size = snp_buffer_copyout_ptr(buffer, &data, 1024);

	HAL_UART_Transmit(huart, data, size, 100);

	snp_buffer_drain(buffer, size);

	return 0;
}


static int32_t bau_shell_get_motor_speed(char **cmd_list, int32_t num, char *res_str, int32_t res_size, void *cb_handle)
{
	snprintf(res_str, res_size - 1, "target: %d, cur value: %d\r\n", motor_get_target(cb_handle), motor_get_cur_value(cb_handle));

	return strlen(res_str) + 1;
}


static int32_t bau_shell_set_motor_speed(char **cmd_list, int32_t num, char *res_str, int32_t res_size, void *cb_handle)
{
	char *target_str = cmd_list[3];
	float target_speed = atoi(target_str);

	motor_set_target(cb_handle, target_speed);

	return 0;
}


/**
 * @brief bau日志信息重定向
 */
static void bau_log_print_if(enum SNP_LOG_TYPE type, char *fmt, ...)
{

}


void libsnp_task_thread_exec(void const *argument)
{
	snp_shell_init();
	snp_set_log_if(bau_log_print_if);

	static char *read_left_motor_speed[] = {"get", "left", "motor", "speed"};
	snp_shell_setup(read_left_motor_speed, 4, bau_shell_get_motor_speed, left_motor);

	static char *read_right_motor_speed[] = {"get", "right", "motor", "speed"};
	snp_shell_setup(read_right_motor_speed, 4, bau_shell_get_motor_speed, right_motor);

	static char *set_left_motor_speed[] = {"set", "left", "motor", SNP_SHELL_PLACEHOLDER};
	snp_shell_setup(set_left_motor_speed, 4, bau_shell_set_motor_speed, left_motor);

	static char *set_right_motor_speed[] = {"set", "right", "motor", SNP_SHELL_PLACEHOLDER};
	snp_shell_setup(set_right_motor_speed, 4, bau_shell_set_motor_speed, right_motor);

	struct SNP *snp_bau_handle = snp_create("bau_monitor", SDK_BAU_MONITOR, SDK_BAU_MONITOR);
	struct SNP_LINK *link = snp_create_physical_node(snp_bau_handle, bau_snp_link_read, bau_snp_link_write, &huart2);

	HAL_UART_Receive_DMA(&huart1, snp_rw_buffer.dma_buffer, sizeof(snp_rw_buffer.dma_buffer));

	do
	{
		osDelay(10);
		snp_exec(snp_bau_handle, 10);
	} while (true);
}



/**
 * @brief snp协议栈任务初始化
 */
void libsnp_app_init(void)
{
	snp_init();

	osThreadDef(libsnp_task_thread, libsnp_task_thread_exec, osPriorityNormal, 0, 4096);
	osThreadCreate(osThread(libsnp_task_thread), NULL);

	// libsnp_task_thread_exec(NULL);
}

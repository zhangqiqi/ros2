#include "libsnp_app.h"

#include "defs.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include <stdio.h>
#include <stdarg.h>

#include "ssnp.h"
#include "ssnp_buffer.h"
#include "ssnp_shell.h"
#include "ssnp_msgs.h"
#include "ssnp_msgs_def.h"

#include "motor.h"

extern struct MOTOR *left_position_ctrl;
extern struct MOTOR *right_position_ctrl;
extern struct MOTOR *left_motor;
extern struct MOTOR *right_motor;

struct DMA_BUFFER_CTRL {
	int32_t read_idx;

	uint8_t dma_buffer[SSNP_RECV_BUFFER_SIZE * 2];
};

static struct DMA_BUFFER_CTRL snp_rw_buffer = {0};

static int32_t bau_ssnp_read(void *handle, struct SSNP_BUFFER *buffer)
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
		if (ssnp_buffer_write(buffer, snp_rw_buffer.dma_buffer, cur_read_cnt) <= 0)
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


static int32_t bau_ssnp_write(void *handle, struct SSNP_BUFFER *buffer)
{
	UART_HandleTypeDef *huart = (UART_HandleTypeDef *)handle;

	uint8_t *data = NULL;

	int32_t size = ssnp_buffer_copyout_ptr(buffer, &data, 1024);

	HAL_UART_Transmit(huart, data, size, 100);

	ssnp_buffer_drain(buffer, size);

	return 0;
}


static int32_t bau_shell_get_motor_speed(char **cmd_list, int32_t num, char *res_str, int32_t res_size, void *cb_handle)
{
	snprintf(res_str, res_size - 1, "target: %d, cur value: %d\r\n", motor_get_target(cb_handle), motor_get_cur_value(cb_handle));

	return strlen(res_str) + 1;
}


static int32_t bau_shell_set_motor_speed(char **cmd_list, int32_t num, char *res_str, int32_t res_size, void *cb_handle)
{
	char *target_str = cmd_list[4];
	float target = atoi(target_str);

	motor_set_target(cb_handle, target);

	return 0;
}


static int32_t bau_shell_get_motor_position(char **cmd_list, int32_t num, char *res_str, int32_t res_size, void *cb_handle)
{
	snprintf(res_str, res_size - 1, "target: %d, cur value: %d\r\n", motor_get_target(cb_handle), motor_get_cur_value(cb_handle));

	return strlen(res_str) + 1;
}


static int32_t bau_shell_set_motor_position(char **cmd_list, int32_t num, char *res_str, int32_t res_size, void *cb_handle)
{
	char *target_str = cmd_list[4];
	float target = atoi(target_str);

	motor_state_clear(cb_handle);
	motor_set_target(cb_handle, target);

	return 0;
}


static int32_t bau_shell_set_both_motor_position(char **cmd_list, int32_t num, char *res_str, int32_t res_size, void *cb_handle)
{
	char *target_str = cmd_list[4];
	float target = atoi(target_str);

	motor_state_clear(left_position_ctrl);
	motor_set_target(left_position_ctrl, target);

	motor_state_clear(right_position_ctrl);
	motor_set_target(right_position_ctrl, -target);

	return 0;
}


/**
 * @brief 轮子电机控制消息
 * @param cb_handle 消息回调引用句柄
 * @param ssnp 消息来源协议栈
 * @param msg 消息体
 * @return int32_t 
 */
int32_t ssnp_wheel_motor_ctrl_msg_proc(void *cb_handle, struct SSNP *ssnp, struct SSNP_FRAME *msg)
{
	struct SMT_WHEEL_MOTOR_CTRL_MSG *_msg = (struct SMT_WHEEL_MOTOR_CTRL_MSG *)msg->payload;

	motor_set_target(left_motor, _msg->left_motor_count);
	motor_set_target(right_motor, -_msg->right_motor_count);

	return 0;
}


void libssnp_task_thread_exec(void const *argument)
{
	ssnp_shell_init();

	struct SSNP *ssnp = ssnp_create();
	ssnp_recv_if_setup(ssnp, &huart2, bau_ssnp_read);
	ssnp_trans_if_setup(ssnp, &huart2, bau_ssnp_write);

	ssnp_msgs_listener_setup(ssnp, SMT_WHEEL_MOTRO_CTRL, ssnp_wheel_motor_ctrl_msg_proc, NULL);

	HAL_UART_Receive_DMA(&huart2, snp_rw_buffer.dma_buffer, sizeof(snp_rw_buffer.dma_buffer));

	do
	{
		osDelay(10);
		ssnp_exec(ssnp);
	} while (true);
}


/**
 * @brief snp协议栈任务初始化
 */
void libsnp_app_init(void)
{
	osThreadDef(libssnp_task_thread, libssnp_task_thread_exec, osPriorityNormal, 0, 4096);
	osThreadCreate(osThread(libssnp_task_thread), NULL);
}

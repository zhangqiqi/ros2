#include "node_motor.h"
#include "node_recv_app.h"
#include "defs.h"


void node_tx_task(void const * argument)
{
//	HAL_StatusTypeDef ret = HAL_OK;
	BaseType_t result;
	unsigned char odometry_data[50]={0};   //���͸����ڵ���̼���������
	MSG_ROS_HEAD head;
	MSG_ROS_ODOM_TYPE ros_odom_type;
	MSG_ROS_CRC crc;
	memset(&head, 0, sizeof(head));
	memset(&ros_odom_type, 0, sizeof(ros_odom_type));
	memset(&crc, 0, sizeof(crc));
//	uint16_t len = 0;
	head.cmd = ROS_CMD_TYPE_ODOM;
	head.len = sizeof(ros_odom_type);
	head.magic =  ROS_MAGIC;
	crc.crc_data = 0x0d0a;	//"\r\n"
	osSemaphoreWait(init_complete, osWaitForever);
	while(1)
	{
		result = xQueueReceive(queue_odom_handle, (void *)&ros_odom_type, 100);
		//osDelay(20);
		if(pdTRUE == result)
		{
			memcpy(odometry_data, &head, sizeof(head));
			memcpy(odometry_data + sizeof(head), &ros_odom_type, sizeof(ros_odom_type));
			memcpy(odometry_data + sizeof(head) + sizeof(ros_odom_type), &crc.crc_data, sizeof(crc.crc_data));
//			len = sizeof(head) + sizeof(ros_odom_type) + sizeof(crc.crc_data);
			/*******uart send********/
			osSemaphoreWait(semnode_tx_cpt, osWaitForever);
			// ret = HAL_UART_Transmit_DMA(&huart2, (uint8_t *)odometry_data, len);
	//		LOGI("actual_left_speed %f actual_right_speed %f ret %d \r\n", ros_odom_type.actual_left_speed, ros_odom_type.actual_right_speed, ret);  
	//		LOGI("ros_odom_type %f %f %f %f %f %f %f", ros_odom_type.x_data, ros_odom_type.y_data, ros_odom_type.theta_data, ros_odom_type.vel_linear,
	//			ros_odom_type.vel_angular, ros_odom_type.actual_left_speed, ros_odom_type.actual_right_speed);
	//		LOGI("ros_cmd_len %d\r\n", len);
	
	//		osDelay(20);

		}
#if 0
		ros_odom_type.x_data = 10.0;
		ros_odom_type.y_data = 11.0;
		ros_odom_type.theta_data = 12.0;
		ros_odom_type.vel_angular = 13.0;
		ros_odom_type.vel_linear = 14.0;
		ros_odom_type.actual_left_speed = 15.0;
		ros_odom_type.actual_right_speed = 16.0;
#endif

	}
}




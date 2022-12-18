#ifndef __NODE_RECV_APP__
#define __NODE_RECV_APP__
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#define MAX_RECV_SIZE	128
#define ROS_MAGIC		0xfa

typedef enum
{
	ROS_CMD_TYPE_COM = 0,
	ROS_CMD_TYPE_MOTOR_SPEED = 2,
	ROS_CMD_TYPE_ODOM = 3,
}ROS_CMD_TYPE;

typedef struct
{
	uint8_t magic;
	uint8_t cmd;
	uint16_t len;
}MSG_ROS_HEAD;

typedef struct
{
	uint16_t crc_data;
}MSG_ROS_CRC;

typedef struct
{
	float right_motor_data; 
	float left_motor_data;
}MSG_ROS_SPEED_TYPE;

typedef struct
{
	float x_data;	//X方向移动的距离
	float y_data;	//Y方向移动的距离
	float theta_data;	//当前角度
	float vel_linear;	//线速度
	float vel_angular;	//角速度	
	float actual_left_speed;	//实际速度左
	float actual_right_speed;	//实际速度右
}MSG_ROS_ODOM_TYPE;


typedef struct
{
	uint8_t *rx_buf;
	uint16_t pWritePtr;
	uint16_t pReadPtr;
	uint16_t rx_size;
}RX_BUF_TYPE;

extern MSG_ROS_SPEED_TYPE ros_speed_data;
uint32_t get_data_from_rx_buf(RX_BUF_TYPE *p_rx_buf, uint8_t *buf, uint16_t buf_len);
uint32_t parse_speed_cmd(RX_BUF_TYPE *p_parse_buf);


#endif /*__NODE_RECV_APP__*/


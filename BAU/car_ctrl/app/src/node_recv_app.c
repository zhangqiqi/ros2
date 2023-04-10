#include "node_recv_app.h"
#include "cli_uart.h"

/***********************************************  说明  *****************************************************************
*
*   1.串口接收
*    （1）内容：小车左右轮速度,单位:mm/s（所有数据都为   float型，float型占4字节）
*    （2）格式：10字节 [右轮速度4字节][左轮速度4字节][结束符"\r\n"2字节]
*		 [起始符1字节][命令号1字节][长度2字节][右轮速度4字节][左轮速度4字节] [结束符"\r\n"2字节]  14字节
*		  0xfa cmd len
*   2.串口发送
*    （1）内容：里程计（x,y坐标、线速度、角速度和方向角，单位依次为：mm,mm,mm/s,rad/s,rad，所有数据都为float型，float型占4字节）
*    （2）格式：21字节 [x坐标4字节][y坐标4字节][方向角4字节][线速度4字节][角速度4字节][结束符"\n"1字节]
*
************************************************************************************************************************/

MSG_ROS_SPEED_TYPE ros_speed_data;

uint32_t parse_speed_cmd(RX_BUF_TYPE *p_parse_buf)
{
	MSG_ROS_HEAD head;
	uint16_t head_len = 0;	
//	uint16_t buf_len = 0;
	uint16_t payload_len = 0;
	uint16_t msg_id = 0;
	head_len = sizeof(head);

	memset(&head, 0, sizeof(head));
	while ((p_parse_buf->pWritePtr - p_parse_buf->pReadPtr) >= head_len)
	{
//		LOGI("func head_len %d\r\n", head_len);
		memcpy(&head, p_parse_buf->rx_buf + p_parse_buf->pReadPtr, head_len);
		if (ROS_MAGIC == head.magic)
		{
//			LOGI("func head.magic %d", head.magic);
			msg_id = head.cmd;
			payload_len = head.len;
			if ((p_parse_buf->pWritePtr - p_parse_buf->pReadPtr) < (payload_len + head_len + 2))
			{
				break;
			}
			if(msg_id == 0x02)  /***速度控制*/
			{
				memcpy(&ros_speed_data, p_parse_buf->rx_buf + p_parse_buf->pReadPtr + head_len, payload_len);
				LOGI("receive ROS_MAGIC head.cmd %d head.len:%d %d %d \r\n", head.cmd, head.len, p_parse_buf->pWritePtr, p_parse_buf->pReadPtr);
				LOGI("func %s left_motor_data %f right_motor_data %f", __FUNCTION__, ros_speed_data.left_motor_data, ros_speed_data.right_motor_data);
			}
			p_parse_buf->pReadPtr += payload_len + head_len + 2;
			
		}
		else
		{
			p_parse_buf->pReadPtr += 1;
		}
	}

	if (p_parse_buf->pReadPtr != 0)
	{
		if ((p_parse_buf->pWritePtr - p_parse_buf->pReadPtr) > 0)
		{
			memmove(p_parse_buf->rx_buf, p_parse_buf->rx_buf + p_parse_buf->pReadPtr, p_parse_buf->pWritePtr - p_parse_buf->pReadPtr);
			p_parse_buf->pWritePtr -= p_parse_buf->pReadPtr;
			p_parse_buf->pReadPtr = 0;
		}
		else
		{
			p_parse_buf->pWritePtr = 0;
			p_parse_buf->pReadPtr = 0;
		}
	}
	LOGI("receive ROS_MAGIC head.cmd %d head.len:%d %d %d \r\n", head.cmd, head.len, p_parse_buf->pWritePtr, p_parse_buf->pReadPtr);	
	
	return 0;
}



uint32_t get_data_from_rx_buf(RX_BUF_TYPE *p_rx_buf, uint8_t *buf, uint16_t buf_len)
{
	uint32_t cpy_len;
	uint32_t cpy_total_len = 0;

	if (buf_len == 0)
	{
		return 0;
	}
	if (p_rx_buf->pWritePtr > p_rx_buf->pReadPtr)
	{
		cpy_len = p_rx_buf->pWritePtr - p_rx_buf->pReadPtr;
		if (cpy_len > buf_len)
		{
			cpy_len = buf_len;
		}
		memcpy(buf, p_rx_buf->rx_buf + p_rx_buf->pReadPtr, cpy_len);
		p_rx_buf->pReadPtr += cpy_len;

		if (p_rx_buf->pReadPtr >= p_rx_buf->rx_size)
		{
			p_rx_buf->pReadPtr -= p_rx_buf->rx_size;
		}
		cpy_total_len += cpy_len;
	}
	else if (p_rx_buf->pWritePtr < p_rx_buf->pReadPtr)   // 12  126
	{
		cpy_len = p_rx_buf->rx_size - p_rx_buf->pReadPtr;
		if (cpy_len > buf_len)
		{
			cpy_len = buf_len;
		}

		memcpy(buf, p_rx_buf->rx_buf + p_rx_buf->pReadPtr, cpy_len);
		p_rx_buf->pReadPtr += cpy_len;//   128 2

		if (p_rx_buf->pReadPtr>= p_rx_buf->rx_size)
		{
			p_rx_buf->pReadPtr -= p_rx_buf->rx_size; //0 
		}
		cpy_total_len += cpy_len;// 2

		if(p_rx_buf->pWritePtr != 0)
		{
			cpy_len = p_rx_buf->pWritePtr; //12
			if (cpy_total_len + cpy_len > buf_len)
			{
				cpy_len = buf_len - cpy_total_len;
			}
			if (cpy_len != 0)
			{
				memcpy(buf + cpy_total_len, p_rx_buf->rx_buf, cpy_len);
				p_rx_buf->pReadPtr += cpy_len;
				if (p_rx_buf->pReadPtr >= p_rx_buf->rx_size)
				{
					p_rx_buf->pReadPtr -= p_rx_buf->rx_size;
				}
			}
			cpy_total_len += cpy_len;
		}
	}

	return cpy_total_len;
}



#include "defs.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "cli_uart.h"
#include "node_recv_app.h"
#include "math.h"



/*主任务的信号量和任务ID*/
osSemaphoreId init_complete;
osSemaphoreId sem_odometry_cal;
osSemaphoreId semlog_tx_cpt;
osSemaphoreId semlog_rx_cpt;
osSemaphoreId semnode_tx_cpt;
osSemaphoreId semnode_rx_cpt;

osMutexId mutex_log;

osThreadId system_task_id, node_recv_task_id, node_tx_task_id;

QueueHandle_t queue_odom_handle;

/*主任务的信号量和任务ID*/


RX_BUF_TYPE recv_buf_ctrl, parse_buf_ctrl;
uint8_t rx_buf[MAX_RECV_SIZE];
uint8_t parse_rx_buf[MAX_RECV_SIZE];



extern float position_x,position_y,oriention,velocity_linear,velocity_angular;         //计算得到的里程计数值


uint32_t Milemeter_L_Motor = 0,Milemeter_R_Motor = 0;
float odometry_right=0,odometry_left=0;//串口得到的左右轮速度


union odometry  //里程计数据共用体
{
	float odoemtry_float;
	unsigned char odometry_char[4];
}x_data,y_data,theta_data,vel_linear,vel_angular,actual_left_speed,actual_right_speed;     //要发布的里程计数据，分别为：X，Y方向移动的距离，当前角度，线速度，角速度

union recieveData  //接收到的数据
{
	float d;    //左右轮速度
	unsigned char data[4];
}leftdata,rightdata;       //接收的左右轮数据

//static unsigned int hRot_Speed2;//电机A平均转速缓存
//static unsigned int hRot_Speed1;//电机B平均转速缓存



uint32_t span;//采集回来的左右轮速度差值



void sem_init(void)
{
	osMutexDef(mu);
	osSemaphoreDef(sem);
	sem_odometry_cal = osSemaphoreCreate(osSemaphore(sem), 1);
	osSemaphoreRelease(sem_odometry_cal);

	semlog_tx_cpt = osSemaphoreCreate(osSemaphore(sem), 1);
	osSemaphoreRelease(semlog_tx_cpt);

	semlog_rx_cpt = osSemaphoreCreate(osSemaphore(sem), 1);
	osSemaphoreRelease(semlog_rx_cpt);

	semnode_tx_cpt = osSemaphoreCreate(osSemaphore(sem), 1);
	osSemaphoreRelease(semnode_tx_cpt);

	semnode_rx_cpt = osSemaphoreCreate(osSemaphore(sem), 1);
	osSemaphoreRelease(semnode_rx_cpt);

	mutex_log = osMutexCreate(osMutex(mu));


}

void queue_init(void)
{
	queue_odom_handle = xQueueCreate(3,sizeof(MSG_ROS_ODOM_TYPE));
}


static uint32_t get_motor_encoder_cnt(TIM_HandleTypeDef *htim)
{
	uint32_t encoder_cnt = 0;
	int32_t dir = 0;
	
		/**< 读取编码器值并清空计数器 */
	/**< 读取编码器方向 */
	dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(htim);
	if ((1 == dir) && (__HAL_TIM_GET_COUNTER(htim) != 0))
	{
		/**< 获取反向编码器值 */
		encoder_cnt = htim->Init.Period - __HAL_TIM_GET_COUNTER(htim);
	}
	else
	{
		/**< 获取正向编码器值 */
		encoder_cnt = __HAL_TIM_GET_COUNTER(htim);
	}
	__HAL_TIM_SET_COUNTER(htim, 0);
	
	return encoder_cnt;
}


void odometry_task(void const * argument)
{
	uint8_t bSpeed_Buffer_Index = 0;//缓存左右轮编码数到数组变量
	int32_t hSpeed_Buffer2[3]={0}, hSpeed_Buffer1[3]={0};//左右轮速度缓存数组
	struct PID Control_left  ={0.9,0.01,0.5,0,0,0,0,0,0};//左轮PID参数，适于新电机4096
	struct PID Control_right ={0.9,0.01,0.5,0,0,0,0,0,0};//右轮PID参数，适于新电机4096
	unsigned long long wtemp3=0;
	unsigned long long wtemp4=0;
	unsigned int hRot_Speed2;//电机A平均转速缓存
	unsigned int hRot_Speed1;//电机B平均转速缓存
	unsigned int Speed2=0; //电机A平均转速 r/min，PID调节
	unsigned int Speed1=0; //电机B平均转速 r/min，PID调节
	float pulse = 0;//电机A PID调节后的PWM值缓存
	float pulse1 = 0;//电机B PID调节后的PWM值缓存
	int right_speed, left_speed;
	static int right_dir = 0, left_dir = 0;      /**< 左右轮方向 */
	int32_t left_pwm_out = 0, right_pwm_out = 0;
	int32_t i = 0, j = 0;
	uint32_t time_cnt = 0;
	float k2=22.0368358	;	   //速度转换比例,转/分钟 mm/s *60* 44 = n/min   一转 44脉冲 853.266392 19.392418 0.440736 9.696209006074964  969.6207771515426 10ms
								//V mm/s = V * 60 * m/min    转换成 脉冲 = V * 60 * 转速比 * 
	MSG_ROS_ODOM_TYPE queue_odom_tx;
	SPEED_PULSE_CNT speed_pluse_cnt;
	memset(&queue_odom_tx, 0, sizeof(queue_odom_tx));
	memset(&speed_pluse_cnt, 0, sizeof(speed_pluse_cnt));	
	osSemaphoreWait(init_complete, osWaitForever);		/*等待任务初始化完成*/
	while(1)
	{
		osSemaphoreWait(sem_odometry_cal, 5);  /*默认10ms释放信号量*/
//		LOGI("Milemeter_R_Motor %d %d \r\n", Milemeter_R_Motor, Milemeter_L_Motor);

//		speed_pluse_cnt.left_cnt = Milemeter_L_Motor;
		
		speed_pluse_cnt.left_cnt = get_motor_encoder_cnt(&htim2);
		
		speed_pluse_cnt.right_cnt = get_motor_encoder_cnt(&htim3);
		
		Milemeter_R_Motor = 0;
		Milemeter_L_Motor = 0;
        /************************ 3 ***************************/
        
        //开始缓存左右轮编码数到数组
		
        hSpeed_Buffer2[bSpeed_Buffer_Index] = speed_pluse_cnt.right_cnt;
        hSpeed_Buffer1[bSpeed_Buffer_Index] = speed_pluse_cnt.left_cnt;

		
        bSpeed_Buffer_Index++;//数组移位

		wtemp4 = 0;
		wtemp3 = 0;
        //缓存左右轮编码数到数组结束判断
        if(bSpeed_Buffer_Index >=3)
        {
            bSpeed_Buffer_Index=0;//缓存左右轮编码数到数组变量清零
        }
            //累加缓存次数内的速度值
		for (i=0;i<3;i++)
		{
			wtemp4 += hSpeed_Buffer2[i];
			wtemp3 += hSpeed_Buffer1[i];
		}
	    
	    //取平均，平均脉冲数单位为 个/s	
		wtemp3 /= (3);
		wtemp4 /= (3); //平均脉冲数 个/s		
//		LOGI("uartsend %lld %lld \r\n", wtemp3, wtemp4);	    
	    //将平均脉冲数单位转为 r/min
		wtemp3 = (wtemp3 * 200)*60/44; 	//5ms 转成   r/min  13  (wtemp3 * 50)*60/(44);
		wtemp4 = (wtemp4 * 200)*60/44; 	//	
//		LOGI("uartsend %lld %lld \r\n", wtemp3, wtemp4);

		hRot_Speed2= ((int16_t)(wtemp4));//平均转速 r/min
		hRot_Speed1= ((int16_t)(wtemp3));//平均转速 r/min
		Speed2=hRot_Speed2;//平均转速 r/min
		Speed1=hRot_Speed1;//平均转速 r/min
//		LOGI("Speed2 %d Speed2 %d \r\n", hRot_Speed2, hRot_Speed1);
        /************************ 4 ***************************/


		//将从串口接收到的速度转换成实际控制小车的速度？还是PWM？
		// 
		right_speed = (int)k2*odometry_right;// + 10000;
		left_speed = (int)k2*odometry_left;// + 10000;
		Control_left.OwenValue = fabs(k2*odometry_left);// + 10000;
		Control_right.OwenValue = fabs(k2*odometry_right);
//		LOGI("Control_right.OwenValue %f", Control_right.OwenValue);
//		LOGI("right_speed %d left_speed %d hRot_Speed1 %d hRot_Speed2:%d\r\n", right_speed, left_speed, hRot_Speed1, hRot_Speed2);      
		span=1*(Speed1-Speed2);//采集回来的左右轮速度差值
		pulse = (pulse + PID_calculate(&Control_right,hRot_Speed2));//PID调节  
		right_pwm_out = (int32_t)pulse;
		right_pwm_out = fabs(right_pwm_out);
		if (right_pwm_out > 8400)
		{
			right_pwm_out = 8400;
		}
		else if (right_pwm_out < 0)
		{
			right_pwm_out = 0;
		}
		//pwm幅度抑制
//        LOGI("right_speed %d left_speed %d\r\n", right_speed, left_speed);
        span=1*(Speed2-Speed1);//采集回来的左右轮速度差值
		pulse1 = (pulse1 + PID_calculate(&Control_left,hRot_Speed1));//PID调节
	    left_pwm_out = (int32_t)pulse1;
		left_pwm_out = fabs(left_pwm_out);
//		LOGI("pulse %f pulse1 %f\r\n", pulse, pulse1); 	
		
		if (left_pwm_out > 8400)
		{
			left_pwm_out = 8400;
		}
		else if (left_pwm_out < 0)
		{
			left_pwm_out = 0;
		}
		if (right_pwm_out > 8400)
		{
			right_pwm_out = 8400;
		}
		else if (right_pwm_out < 0)
		{
			right_pwm_out = 0;
		}		
//		LOGI("right_pwm_out %d left_pwm_out %d\r\n", right_pwm_out, left_pwm_out);     

#if 0
			htim4.Instance->CCR3 = 0;
			htim4.Instance->CCR4 = 840;	
			htim4.Instance->CCR1 = 0;
			htim4.Instance->CCR2 = 840;
#else		
		if (right_speed > 0)    /*正传*/
		{
			htim4.Instance->CCR3 = 0;
			htim4.Instance->CCR4 = right_pwm_out;	

		}
		else if (right_speed < 0) /*反传*/
		{
			htim4.Instance->CCR3 = right_pwm_out;
			htim4.Instance->CCR4 = 0;
		}
		else
		{
			htim4.Instance->CCR3 = 0;
			htim4.Instance->CCR4 = 0;	
		}
	
		if (left_speed > 0)
		{
			htim4.Instance->CCR1 = 0;
			htim4.Instance->CCR2 = left_pwm_out;
		}
		else if (left_speed < 0)
		{
			htim4.Instance->CCR1 = left_pwm_out;
			htim4.Instance->CCR2 = 0;
		}
		else
		{
			htim4.Instance->CCR1 = 0;
			htim4.Instance->CCR2 = 0;

		}
#endif 

//		LOGI("right_pwm_out %d left_pwm_out %d\r\n", right_pwm_out, left_pwm_out);  
		odometry(&speed_pluse_cnt, &queue_odom_tx);//计算里程计
//		LOGI("odom %d %d",speed_pluse_cnt.left_cnt,speed_pluse_cnt.right_cnt);
//		LOGI("queue_odom %f %f %f %f %f", queue_odom_tx.x_data, queue_odom_tx.y_data, queue_odom_tx.vel_linear, queue_odom_tx.vel_angular, queue_odom_tx.theta_data);
		if (time_cnt++ > 5)
		{
			time_cnt = 0;
			xQueueSend(queue_odom_handle, (void *)&queue_odom_tx, 0);
		}

//		LOGI("position_x %f position_y %f oriention %f velocity_linear %f velocity_angular %f\r\n",
//								position_x, position_x, oriention, velocity_linear, velocity_angular);

	}
}

void node_recv_task(void const * argument)
{
	uint32_t cnt = 0, flag = 0;
	RX_BUF_TYPE *recv_buf;
	RX_BUF_TYPE	*parse_buf;
	uint16_t recv_len = 0;
	recv_buf = &recv_buf_ctrl;
	parse_buf = &parse_buf_ctrl;

	parse_buf->rx_buf = parse_rx_buf;
	parse_buf->pReadPtr = 0;
	parse_buf->pWritePtr = 0;
	parse_buf->rx_size = 128;
	
	recv_buf->rx_buf = rx_buf;
	recv_buf->pWritePtr= 0;
	recv_buf->pReadPtr= 0;
	recv_buf->rx_size = 128;
	memset(&ros_speed_data, 0, sizeof(ros_speed_data));
	memset(rx_buf, 1, sizeof(rx_buf));
	uint32_t t = 0;
	HAL_UART_Receive_DMA(&huart2, recv_buf->rx_buf, recv_buf->rx_size);
	osSemaphoreWait(init_complete, osWaitForever);		/*等待任务初始化完成*/	
	while(1)
	{
		osSemaphoreWait(semnode_rx_cpt, 1);
		recv_buf->pWritePtr = huart2.RxXferSize - huart2.hdmarx->Instance->NDTR;	/*总长度-剩余长度 = 接收长度*/
//		LOGI("..... parse_buf->pWritePtr:%d recv_buf->pWritePtr:%d huart1.RxXferSize:%d huart1.hdmarx->Instance->NDTR:%d recv_len:%d parse_buf->pReadPtr:%d recv_buf->pReadPtr:%d",
//								parse_buf->pWritePtr, recv_buf->pWritePtr, huart1.RxXferSize, huart1.hdmarx->Instance->NDTR, recv_len, parse_buf->pReadPtr, recv_buf->pReadPtr);
		recv_len = get_data_from_rx_buf(recv_buf, parse_buf->rx_buf + parse_buf->pWritePtr, parse_buf->rx_size - parse_buf->pWritePtr);
		if(recv_len == 0)
		{
			//return;
		}
		else
		{
			parse_buf->pWritePtr += recv_len;
	//		LOGI("..... parse_buf->pWritePtr:%d recv_buf->pWritePtr:%d huart1.RxXferSize:%d huart1.hdmarx->Instance->NDTR:%d recv_len:%d parse_buf->pReadPtr:%d recv_buf->pReadPtr:%d",
	//								parse_buf->pWritePtr, recv_buf->pWritePtr, huart1.RxXferSize, huart1.hdmarx->Instance->NDTR, recv_len, parse_buf->pReadPtr, recv_buf->pReadPtr);

	//		LOGI("..... %d %d", parse_buf->pWritePtr, recv_buf->pWritePtr);
			parse_speed_cmd(parse_buf);
	        //接收左右轮速度
			LOGI("odometry_right %f odometry_left %f", ros_speed_data.right_motor_data, ros_speed_data.left_motor_data);
			//odometry_right = ros_speed_data.right_motor_data;
			//odometry_left = ros_speed_data.left_motor_data;
		}
		

	}
}

void system_task(void const * argument)
{
	int32_t i = 0;
	uint8_t send_buf[128];
	memset(send_buf, 1, sizeof(send_buf));

	sem_init();
	queue_init();
	osThreadDef(odom, odometry_task, osPriorityAboveNormal, 0, 1024);
	system_task_id = osThreadCreate(osThread(odom), NULL);	

	osThreadDef(node, node_recv_task, osPriorityNormal, 0, 1024);
	node_recv_task_id = osThreadCreate(osThread(node), NULL);

	osThreadDef(node_tx, node_tx_task, osPriorityNormal, 0, 1024);

	node_tx_task_id = osThreadCreate(osThread(node_tx), NULL);

	HAL_TIM_Base_Start_IT(&htim6);
	
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	LOGI("reboot\r\n");
#if 0	
	htim4.Instance->CCR3 = 0;
	htim4.Instance->CCR4 = 0;
	htim3.Instance->CCR1 = 0;
	htim3.Instance->CCR2 = 0;
#endif	
	for(i = 0; i < 5; i++)
	{
		osSemaphoreRelease(init_complete);
	}
	
	while(1)
	{

//		HAL_UART_Transmit_DMA(&huart1, send_buf, 10);
		osDelay(5000);
		LOGI("func %s\r\n", __FUNCTION__);
	}
}

void main_app(void)
{
	/*初始化任务统一开启信号量*/
	osSemaphoreDef(sem);
	init_complete = osSemaphoreCreate(osSemaphore(sem), 1);
	
	osThreadDef(sys_init, system_task, osPriorityBelowNormal, 0, 512);
	system_task_id = osThreadCreate(osThread(sys_init), NULL);	

}

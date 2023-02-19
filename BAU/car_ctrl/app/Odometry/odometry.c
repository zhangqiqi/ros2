#include "odometry.h"
#include "node_motor.h"
#include "cli_uart.h"
#include <math.h>


/***********************************************  输出  *****************************************************************/

float position_x=0,position_y=0,oriention=0,velocity_linear=0,velocity_angular=0;

/***********************************************  输入  *****************************************************************/

extern float odometry_right,odometry_left;//串口得到的左右轮速度

/***********************************************  变量  *****************************************************************/

float wheel_interval= 170.0859f;//    272.0f;        //  1.0146
//float wheel_interval=276.089f;    //轴距校正值=原轴距/0.987

float multiplier=1.0f;           //倍频数 4
float deceleration_ratio=75.0f;  //减速比 90
float wheel_diameter=65.0f;     //轮子直径，单位mm
float pi_1_2=1.570796f;			 //π/2
float pi=3.141593f;              //π
float pi_3_2=4.712389f;			 //π*3/2
float pi_2_1=6.283186f;			 //π*2
float dt=0.005f;                 //采样时间间隔5ms  20ms
float line_number=44.0f;       //码盘线数 4096




//float oriention_1=0;

//uint8_t once=1;

/****************************************************************************************************************/

#if 0
//里程计计算函数
//输入 实时转速
void odometry(float right,float left)
{	
	if(once)  //常数仅计算一次
	{
		const_frame=wheel_diameter*pi/(line_number*multiplier*deceleration_ratio);  //v = 2*pi *r*转速/转速比  mm/s
		const_angle=const_frame/wheel_interval;
		once=0;
	}
    
	distance_sum = 0.5f*(right+left);//在很短的时间内，小车行驶的路程为两轮速度和    r/min
	distance_diff = right-left;//在很短的时间内，小车行驶的角度为两轮速度差

    //根据左右轮的方向，纠正短时间内，小车行驶的路程和角度量的正负
//     odometry_left = 1;
//     odometry_right = 1;
	if((odometry_right>0)&&(odometry_left>0))            //左右均正
	{
		delta_distance = distance_sum;
		delta_oriention = distance_diff;
	}
	else if((odometry_right<0)&&(odometry_left<0))       //左右均负
	{
		delta_distance = -distance_sum;
		delta_oriention = -distance_diff;
	}
	else if((odometry_right<0)&&(odometry_left>0))       //左正右负
	{
		delta_distance = -distance_diff;
		delta_oriention = -2.0f*distance_sum;		
	}
	else if((odometry_right>0)&&(odometry_left<0))       //左负右正
	{
		delta_distance = distance_diff;
		delta_oriention = 2.0f*distance_sum;
	}
	else
	{
		delta_distance=0;
		delta_oriention=0;
	}
    
	oriention_interval = delta_oriention * const_angle;//采样时间内走的角度	
	oriention = oriention + oriention_interval;//计算出里程计方向角
	oriention_1 = oriention + 0.5f * oriention_interval;//里程计方向角数据位数变化，用于三角函数计算
	
    sin_ = sin(oriention_1);//计算出采样时间内y坐标
	cos_ = cos(oriention_1);//计算出采样时间内x坐标
	
    position_x = position_x + delta_distance * cos_ * const_frame;//计算出里程计x坐标
	position_y = position_y + delta_distance * sin_ * const_frame;//计算出里程计y坐标
    
	velocity_linear = delta_distance*const_frame / dt;//计算出里程计线速度
	velocity_angular = oriention_interval / dt;//计算出里程计角速度
	
    //方向角角度纠正
	if(oriention > pi)
	{
		oriention -= pi_2_1;
	}
	else
	{
		if(oriention < -pi)
		{
			oriention += pi_2_1;
		}
	}
}
#else
//里程计计算函数
//输入 实时转速
void odometry(SPEED_PULSE_CNT *pluse_cnt, MSG_ROS_ODOM_TYPE *odom_type)
{	
	static float oriention_1 = 0;
	static uint8_t once = 1;
	static double const_frame = 0.0f, const_angle = 0.0f, distance_sum = 0.0f;
	float distance_diff = 0;
	float delta_distance = 0, delta_oriention = 0;	 //采样时间间隔内运动的距离
	float oriention_interval = 0;  //dt时间内方向变化值
	float sin_=0;		 //角度计算值
	float cos_=0;

	if (once)  //常数仅计算一次
	{
		const_frame = wheel_diameter*pi/(line_number*multiplier*deceleration_ratio);  //v = 2*pi *r*转速/转速比  mm/s
		const_angle = const_frame/wheel_interval;
		once=0;
		LOGI("one %lf %lf\r\n", const_angle, const_frame);
	}
//    LOGI("pluse_cnt %d %d", pluse_cnt->right_cnt, pluse_cnt->left_cnt);
	distance_sum = 0.5f * ((int32_t)pluse_cnt->right_cnt + (int32_t)pluse_cnt->left_cnt);//在很短的时间内，小车行驶的路程为两轮速度和    r/min
	distance_diff = ((int32_t)pluse_cnt->right_cnt - (int32_t)pluse_cnt->left_cnt);//在很短的时间内，小车行驶的角度为两轮速度差
	if(distance_diff > 100)
	{
//		distance_diff = 0;
	}

    //根据左右轮的方向，纠正短时间内，小车行驶的路程和角度量的正负
    
	if((odometry_right>0)&&(odometry_left>0))            //左右均正
	{
		delta_distance = distance_sum;
		delta_oriention = distance_diff;
		//LOGI("pluse_cnt %d %d", pluse_cnt->right_cnt, pluse_cnt->left_cnt);
		//LOGI("delta %f %f\r\n", distance_sum, distance_diff);
		//LOGI("delta %f %f\r\n", delta_distance, delta_oriention);
	}
	else if((odometry_right<0)&&(odometry_left<0))       //左右均负
	{
		delta_distance = -distance_sum;
		delta_oriention = -distance_diff;
	}
	else if((odometry_right<0)&&(odometry_left>0))       //左正右负
	{
		delta_distance = -distance_diff;
		delta_oriention = -2.0f*distance_sum;		
	}
	else if((odometry_right>0)&&(odometry_left<0))       //左负右正
	{
		delta_distance = distance_diff;
		delta_oriention = 2.0f*distance_sum;
	}
	else
	{
		delta_distance=0;
		delta_oriention=0;
	}
    
	oriention_interval = delta_oriention * const_angle;//采样时间内走的角度
//	LOGI("int %f\r\n", oriention_interval);
	odom_type->theta_data = odom_type->theta_data + oriention_interval;//计算出里程计方向角
	oriention_1 = odom_type->theta_data + 0.5f * oriention_interval;//里程计方向角数据位数变化，用于三角函数计算
//	LOGI("oriention_1 %f\r\n", oriention_1);
	
    sin_ = sin(oriention_1);//计算出采样时间内y坐标
	cos_ = cos(oriention_1);//计算出采样时间内x坐标
	
//	LOGI("int %f %f\r\n", sin_, cos_);

	odom_type->x_data = odom_type->x_data + delta_distance * cos_ * const_frame;//计算出里程计x坐标
	odom_type->y_data = odom_type->y_data + delta_distance * sin_ * const_frame;//计算出里程计y坐标
//	LOGI("int %f %f\r\n", odom_type->x_data, odom_type->y_data);

	odom_type->vel_linear = delta_distance*const_frame / dt;//计算出里程计线速度
	odom_type->vel_angular = oriention_interval / dt;//计算出里程计角速度
	
	if (odom_type->theta_data > pi)
	{
		odom_type->theta_data -= pi_2_1;
	}
	else
	{
		if (odom_type->theta_data < -pi)
		{
			odom_type->theta_data += pi_2_1;
		}

	}
	
	
	odom_type->actual_left_speed = ((float)pluse_cnt->left_cnt) * 200* pi * wheel_diameter / (deceleration_ratio *line_number) ;
	odom_type->actual_right_speed = ((float)pluse_cnt->right_cnt) * 200* pi * wheel_diameter / (deceleration_ratio *line_number);	
}

#endif


#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include <math.h>
#include <stdlib.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "node_recv_app.h"
#include "node_motor.h"

//#define ODOMETRY_LENGTH 12;
extern float multiplier;           //倍频数 4
extern float deceleration_ratio;  //减速比 90
extern float multiplier;           //倍频数 4
extern float pi_1_2;			 //π/2
extern float pi;              //π
extern float pi_3_2;			 //π*3/2
extern float pi_2_1;			 //π*2
extern float dt;                 //采样时间间隔5ms  20ms
extern float wheel_diameter;     //轮子直径，单位mmvoid odometry(float right,float left);//里程计计算函数
extern float line_number;

extern void odometry(SPEED_PULSE_CNT *pluse_cnt, MSG_ROS_ODOM_TYPE *odom_type);
#endif

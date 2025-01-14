#ifndef __MPU6050_H__
#define __MPU6050_H__

 #include "stm32f4xx.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * 加速度计采样值 m/s²
*/
struct MPU6050_ACCEL {
	float x;
	float y;
	float z;
};


/**
 * 陀螺仪采样值 单位 °/s
*/
struct MPU6050_GYRO {
	float x;
	float y;
	float z;
};


struct MPU6050;

struct MPU6050 *mpu6050_init(I2C_HandleTypeDef *handle);

int32_t mpu6050_exec(struct MPU6050 *handle, struct MPU6050_ACCEL *accel, struct MPU6050_GYRO *gyro);

#ifdef __cplusplus
}
#endif

#endif

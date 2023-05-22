#ifndef __MPU6050_H__
#define __MPU6050_H__

 #include "stm32f4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

struct MPU6050;

struct MPU6050 *mpu6050_init(I2C_HandleTypeDef *handle);

int32_t mpu6050_exec(struct MPU6050 *handle);

#ifdef __cplusplus
}
#endif

#endif

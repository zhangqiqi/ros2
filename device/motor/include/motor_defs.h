#ifndef __DEVICE_MOTOR_INCLUDE_MOTOR_DEFS_H__
#define __DEVICE_MOTOR_INCLUDE_MOTOR_DEFS_H__


#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <memory.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOROT_ENCODER_CNT_NUM (3)      /**< 电机编码器值历史记录个数s */

typedef int32_t (*MOTOR_READ_ENCODER)(void *handle);

#ifdef __cplusplus
}
#endif


#endif // __DEVICE_MOTOR_INCLUDE_MOTOR_DEFS_H__
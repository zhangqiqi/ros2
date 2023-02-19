#ifndef __DEVICE_MOTOR_INCLUDE_MOTOR_H__
#define __DEVICE_MOTOR_INCLUDE_MOTOR_H__

#include "motor_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

struct MOTOR;

int32_t motor_set_encoder_read_if(struct MOTOR *motor, void *encoder_handle, MOTOR_READ_ENCODER read_if);

void motor_exec(struct MOTOR *motor);


#ifdef __cplusplus
}
#endif


#endif // __DEVICE_MOTOR_INCLUDE_MOTOR_H__

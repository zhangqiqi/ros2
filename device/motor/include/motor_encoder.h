#ifndef __DEVICE_MOTOR_INCLUDE_MOTOR_ENCODER_H__
#define __DEVICE_MOTOR_INCLUDE_MOTOR_ENCODER_H__

#include "motor_defs.h"

#ifdef __cplusplus
extern "C" {
#endif


struct MOTOR_COUNTER;

float motor_counter_get_last_value(struct MOTOR_COUNTER *counter);

float motor_counter_update(struct MOTOR_COUNTER *counter);

#ifdef __cplusplus
}
#endif

#endif // __DEVICE_MOTOR_INCLUDE_MOTOR_ENCODER_H__

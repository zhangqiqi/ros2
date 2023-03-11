#ifndef __DEVICE_MOTOR_INCLUDE_MOTOR_ENCODER_H__
#define __DEVICE_MOTOR_INCLUDE_MOTOR_ENCODER_H__

#include "motor_defs.h"

#ifdef __cplusplus
extern "C" {
#endif


struct MOTOR_ENCODER;

float motor_encoder_update(struct MOTOR_ENCODER *encoder);

#ifdef __cplusplus
}
#endif

#endif // __DEVICE_MOTOR_INCLUDE_MOTOR_ENCODER_H__

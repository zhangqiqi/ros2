#ifndef __BAU_CAR_CTRL_APP_INC_LIBSNP_APP_H__
#define __BAU_CAR_CTRL_APP_INC_LIBSNP_APP_H__

#include <stdlib.h>

#include "ssnp_msgs_def.h"

#ifdef __cplusplus
extern "C" {
#endif


void libsnp_app_init(void);

void libssnp_send_mpu6050_data(struct SMT_MPU6050_DATA_PUSH_MSG *msg);

#ifdef __cplusplus
}
#endif

#endif //__BAU_CAR_CTRL_APP_INC_LIBSNP_APP_H__

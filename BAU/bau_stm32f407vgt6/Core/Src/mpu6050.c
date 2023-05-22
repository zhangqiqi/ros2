#include "mpu6050.h"

#define MPU6050_DEV_ADDR (0xD0)


#define MPU6050_SMPLRT_DIV (0x19)
#define MPU6050_SAMPLE_RATE(rate) ((1000 / (rate)) - 1)

#define MPU6050_CONFIG (0x1a)
#define MPU6050_FIFO_EN (0x23)


#define MPU6050_GYRO_CONFIG (0x1b)
#define MPU6050_GYRO_FS_SEL_0 (0 << 3)      /**< ± 250 °/s */
#define MPU6050_GYRO_FS_SEL_1 (1 << 3)      /**< ± 500 °/s */
#define MPU6050_GYRO_FS_SEL_2 (2 << 3)      /**< ± 1000 °/s */
#define MPU6050_GYRO_FS_SEL_3 (3 << 3)      /**< ± 2000 °/s */


#define MPU6050_ACCEL_CONFIG (0x1c)
#define MPU6050_ACCEL_AFS_SEL_0 (0 << 3)      /**< ± 2g */
#define MPU6050_ACCEL_AFS_SEL_1 (1 << 3)      /**< ± 4g */
#define MPU6050_ACCEL_AFS_SEL_2 (2 << 3)      /**< ± 8g */
#define MPU6050_ACCEL_AFS_SEL_3 (3 << 3)      /**< ± 16g */

#define MPU6050_INT_PIN_CFG (0x37)
#define MPU6050_INT_ENABLE (0x38)

#define MPU6050_USER_CTRL (0x6a)
#define MPU6050_PWR_MGMT_1 (0x6b)
#define MPU6050_PWR_MGMT_2 (0x6c)

#define MPU6050_WHO_AM_I (0x75)

struct MPU6050 {
	I2C_HandleTypeDef *i2c;
};

static struct MPU6050 __mpu6050_instance;


HAL_StatusTypeDef mpu6050_write_byte(struct MPU6050 *handle, uint8_t addr, uint8_t data)
{
	HAL_StatusTypeDef ret = HAL_OK;

	ret = HAL_I2C_Master_Transmit(handle->i2c, MPU6050_DEV_ADDR, &addr, sizeof(addr), 100);
	if (HAL_OK != ret)
	{
		return ret;
	}

	ret = HAL_I2C_Master_Transmit(handle->i2c, MPU6050_DEV_ADDR, &data, sizeof(data), 100);
	if (HAL_OK != ret)
	{
		return ret;
	}

	return ret;
}

HAL_StatusTypeDef mpu6050_read_byte(struct MPU6050 *handle, uint8_t addr, uint8_t *data)
{
	HAL_StatusTypeDef ret = HAL_OK;

	ret = HAL_I2C_Master_Transmit(handle->i2c, MPU6050_DEV_ADDR, &addr, sizeof(addr), 100);
	if (HAL_OK != ret)
	{
		return ret;
	}

	ret = HAL_I2C_Master_Receive(handle->i2c, MPU6050_DEV_ADDR, data, 1, 100);
	if (HAL_OK != ret)
	{
		return ret;
	}

	return ret;
}


struct MPU6050 *mpu6050_init(I2C_HandleTypeDef *handle)
{
	__mpu6050_instance.i2c = handle;

	mpu6050_write_byte(&__mpu6050_instance, MPU6050_PWR_MGMT_1, 0x80);
	osDelay(100);
	mpu6050_write_byte(&__mpu6050_instance, MPU6050_PWR_MGMT_1, 0x00);
	mpu6050_write_byte(&__mpu6050_instance, MPU6050_GYRO_CONFIG, MPU6050_GYRO_FS_SEL_3);
	mpu6050_write_byte(&__mpu6050_instance, MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_AFS_SEL_0);
	mpu6050_write_byte(&__mpu6050_instance, MPU6050_SMPLRT_DIV, MPU6050_SAMPLE_RATE(50));
	mpu6050_write_byte(&__mpu6050_instance, MPU6050_CONFIG, 3);
	mpu6050_write_byte(&__mpu6050_instance, MPU6050_INT_ENABLE, 0);
	mpu6050_write_byte(&__mpu6050_instance, MPU6050_USER_CTRL, 0);
	mpu6050_write_byte(&__mpu6050_instance, MPU6050_FIFO_EN, 0);
	mpu6050_write_byte(&__mpu6050_instance, MPU6050_INT_PIN_CFG, 0);

	uint8_t whoami;
	HAL_StatusTypeDef ret = mpu6050_read_byte(&__mpu6050_instance, MPU6050_WHO_AM_I, &whoami);
	if (HAL_OK != ret || (MPU6050_DEV_ADDR >> 1) != whoami)
	{
		return NULL;
	}

	mpu6050_write_byte(&__mpu6050_instance, MPU6050_PWR_MGMT_1, 0x01);
	mpu6050_write_byte(&__mpu6050_instance, MPU6050_PWR_MGMT_2, 0x00);
	mpu6050_write_byte(&__mpu6050_instance, MPU6050_SMPLRT_DIV, MPU6050_SAMPLE_RATE(50));

	return &__mpu6050_instance;
}


int32_t mpu6050_exec(struct MPU6050 *handle)
{
	HAL_StatusTypeDef ret = HAL_OK;

	return ret;
}

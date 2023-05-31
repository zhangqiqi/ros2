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

#define MPU6050_ACCEL_XOUT_H (0x3b)
#define MPU6050_GYRO_XOUT_H (0x43)

#define MPU6050_USER_CTRL (0x6a)
#define MPU6050_PWR_MGMT_1 (0x6b)
#define MPU6050_PWR_MGMT_2 (0x6c)

#define MPU6050_WHO_AM_I (0x75)

struct MPU6050 {
	I2C_HandleTypeDef *i2c;

	/**< config registers */
	uint8_t whoami;
	uint8_t gyro_config;
	uint8_t accel_config;
	uint8_t smplrt_div;
	uint8_t config;
	uint8_t int_enable;
	uint8_t user_ctrl;
	uint8_t fifo_en;
	uint8_t int_pin_cfg;
};

static struct MPU6050 __mpu6050_instance;


HAL_StatusTypeDef mpu6050_write_byte(struct MPU6050 *handle, uint8_t addr, uint8_t data)
{
	return HAL_I2C_Mem_Write(handle->i2c, MPU6050_DEV_ADDR, addr, sizeof(addr), &data, sizeof(data), 100);
}


HAL_StatusTypeDef mpu6050_read_bytes(struct MPU6050 *handle, uint8_t addr, uint8_t *data, int32_t size)
{
	return HAL_I2C_Mem_Read(handle->i2c, MPU6050_DEV_ADDR, addr, sizeof(addr), data, size, 100);
}


static void mpu6050_config_sync(struct MPU6050 *handle)
{
	HAL_StatusTypeDef ret = HAL_OK;

	ret = mpu6050_read_bytes(handle, MPU6050_WHO_AM_I, &handle->whoami, 1);
	ret = mpu6050_read_bytes(handle, MPU6050_GYRO_CONFIG, &handle->gyro_config, 1);
	ret = mpu6050_read_bytes(handle, MPU6050_ACCEL_CONFIG, &handle->accel_config, 1);
	ret = mpu6050_read_bytes(handle, MPU6050_SMPLRT_DIV, &handle->smplrt_div, 1);
	ret = mpu6050_read_bytes(handle, MPU6050_CONFIG, &handle->config, 1);
	ret = mpu6050_read_bytes(handle, MPU6050_INT_ENABLE, &handle->int_enable, 1);
	ret = mpu6050_read_bytes(handle, MPU6050_USER_CTRL, &handle->user_ctrl, 1);
	ret = mpu6050_read_bytes(handle, MPU6050_FIFO_EN, &handle->fifo_en, 1);
	ret = mpu6050_read_bytes(handle, MPU6050_INT_PIN_CFG, &handle->int_pin_cfg, 1);

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

	uint8_t whoami = 0;
	HAL_StatusTypeDef ret = mpu6050_read_bytes(&__mpu6050_instance, MPU6050_WHO_AM_I, &whoami, 1);
	if (HAL_OK != ret || (MPU6050_DEV_ADDR >> 1) != whoami)
	{
		return NULL;
	}

	mpu6050_write_byte(&__mpu6050_instance, MPU6050_PWR_MGMT_1, 0x01);
	mpu6050_write_byte(&__mpu6050_instance, MPU6050_PWR_MGMT_2, 0x00);
	mpu6050_write_byte(&__mpu6050_instance, MPU6050_SMPLRT_DIV, MPU6050_SAMPLE_RATE(50));

	mpu6050_config_sync(&__mpu6050_instance);

	return &__mpu6050_instance;
}

static HAL_StatusTypeDef mpu6050_read_accel(struct MPU6050 *handle, struct MPU6050_ACCEL *accel)
{
	uint8_t accel_bytes[6] = {0};

	HAL_StatusTypeDef ret = mpu6050_read_bytes(handle, MPU6050_ACCEL_XOUT_H, accel_bytes, sizeof(accel_bytes));
	if (HAL_OK == ret)
	{
		accel->x = (accel_bytes[0] << 8) + accel_bytes[1];
		accel->y = (accel_bytes[2] << 8) + accel_bytes[3];
		accel->z = (accel_bytes[4] << 8) + accel_bytes[5];
	}

	return ret;
}


static HAL_StatusTypeDef mpu6050_read_gyro(struct MPU6050 *handle, struct MPU6050_GYRO *gyro)
{
	uint8_t gyro_bytes[6] = {0};

	HAL_StatusTypeDef ret = mpu6050_read_bytes(handle, MPU6050_GYRO_XOUT_H, gyro_bytes, sizeof(gyro_bytes));
	if (HAL_OK == ret)
	{
		gyro->x = (gyro_bytes[0] << 8) + gyro_bytes[1];
		gyro->y = (gyro_bytes[2] << 8) + gyro_bytes[3];
		gyro->z = (gyro_bytes[4] << 8) + gyro_bytes[5];
	}

	return ret;
}


int32_t mpu6050_exec(struct MPU6050 *handle, struct MPU6050_ACCEL *accel, struct MPU6050_GYRO *gyro)
{
	HAL_StatusTypeDef ret = HAL_OK;

	mpu6050_read_accel(handle, accel);
	mpu6050_read_gyro(handle, gyro);

	return ret;
}

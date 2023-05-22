#include "lis302dl.h"

/* Read/Write command */
#define READWRITE_CMD              ((uint8_t)0x80) 
/* Multiple byte read/write command */ 
#define MULTIPLEBYTE_CMD           ((uint8_t)0x40)

/**
 * lis302设备管理结构体
*/
struct LIS302DL {
	uint8_t power_mode;
	uint8_t output_data_rate;
	uint8_t axes_enable;
	uint8_t full_scale;
	uint8_t self_test;

	void *spi;      /**< 设备通讯操作句柄 */
	LIS302DL_READ_IF read;
	LIS302DL_WRITE_IF write;
};


static struct LIS302DL single_instance = {
	.power_mode = LIS302DL_LOWPOWERMODE_ACTIVE,
	.output_data_rate = LIS302DL_DATARATE_100,
	.axes_enable = LIS302DL_X_ENABLE | LIS302DL_Y_ENABLE | LIS302DL_Z_ENABLE,
	.full_scale = LIS302DL_FULLSCALE_2_3,
	.self_test = LIS302DL_SELFTEST_P | LIS302DL_SELFTEST_M
};


/**
 * @brief 设备初始化
*/
struct LIS302DL *lis302dl_init(void *spi, LIS302DL_READ_IF read_if, LIS302DL_WRITE_IF write_if)
{
	uint8_t ctrl = 0x00;

	single_instance.spi = spi;
	single_instance.read = read_if;
	single_instance.write = write_if;

	ctrl = (uint8_t)(	single_instance.power_mode |
						single_instance.output_data_rate |
						single_instance.axes_enable |
						single_instance.full_scale |
						single_instance.self_test
	);

	lis302dl_write(&single_instance, &ctrl, LIS302DL_CTRL_REG1_ADDR, sizeof(ctrl));
	return &single_instance;
}


/**
 * @brief 
 * @param handle 设备操作句柄
 * @param latch_request
 * @param single_click_axes
 * @param double_click_axes
*/
void lis302dl_interrupt_config(struct LIS302DL *handle, uint8_t latch_request, uint8_t single_click_axes, uint8_t double_click_axes)
{
	uint8_t ctrl = 0x00;

	lis302dl_read(handle, &ctrl, LIS302DL_CLICK_CFG_REG_ADDR, 1);
	ctrl = (uint8_t)(latch_request | single_click_axes | double_click_axes);
	lis302dl_write(handle, &ctrl, LIS302DL_CLICK_CFG_REG_ADDR, 1);
}


void lis302dl_highpass_filter_config(struct LIS302DL *handle, uint8_t data_selection, uint8_t cutoff_freq, uint8_t interrupt)
{
	uint8_t ctrl = 0x00;
	lis302dl_read(handle, &ctrl, LIS302DL_CTRL_REG2_ADDR, 1);
	ctrl &= (uint8_t)~(LIS302DL_FILTEREDDATASELECTION_OUTPUTREGISTER |
						LIS302DL_HIGHPASSFILTER_LEVEL_3|
						LIS302DL_HIGHPASSFILTERINTERRUPT_1_2
	);
	ctrl |= (uint8_t)(data_selection | cutoff_freq | interrupt);

	lis302dl_write(handle, &ctrl, LIS302DL_CTRL_REG2_ADDR, 1);
}


void lis302dl_lowpower_cmd(struct LIS302DL *handle, uint8_t lowpower_mode)
{
	uint8_t tmpreg = 0x00;

	lis302dl_read(handle, &tmpreg, LIS302DL_CTRL_REG1_ADDR, 1);
	tmpreg |= lowpower_mode;

	lis302dl_write(handle, &tmpreg, LIS302DL_CTRL_REG1_ADDR, 1);
}


void lis302dl_fullscale_cmd(struct LIS302DL *handle, uint8_t fullscale_value)
{
	uint8_t tmpreg;

	lis302dl_read(handle, &tmpreg, LIS302DL_CTRL_REG1_ADDR, 1);
	
	tmpreg &= (uint8_t)~LIS302DL_FULLSCALE_9_2;
	tmpreg |= fullscale_value;

	lis302dl_write(handle, &tmpreg, LIS302DL_CTRL_REG1_ADDR, 1);
}

void lis302dl_datarate_cmd(struct LIS302DL *handle, uint8_t datarate_value)
{
	uint8_t tmpreg = 0x00;

	lis302dl_read(handle, &tmpreg, LIS302DL_CTRL_REG1_ADDR, 1);
	tmpreg &= (uint8_t)~LIS302DL_DATARATE_400;
	tmpreg |= datarate_value;

	lis302dl_write(handle, &tmpreg, LIS302DL_CTRL_REG1_ADDR, 1);
}

void lis302_reboot_cmd(struct LIS302DL *handle)
{
	uint8_t tmpreg;

	lis302dl_read(handle, &tmpreg, LIS302DL_CTRL_REG2_ADDR, 1);
	tmpreg |= LIS302DL_BOOT_REBOOTMEMORY;

	lis302dl_write(handle, &tmpreg, LIS302DL_CTRL_REG2_ADDR, 1);
}

void lis302dl_read_acc(struct LIS302DL *handle, int32_t *out)
{
	uint8_t buffer[6] = {0};
	uint8_t ctrl = 0x00, i = 0x00;

	lis302dl_read(handle, &ctrl, LIS302DL_CTRL_REG1_ADDR, 1);
	lis302dl_read(handle, buffer, LIS302DL_OUT_X_ADDR, 6);

	switch (ctrl & 0x20)
	{
	case 0x00:
		for(i=0; i<0x03; i++)
		{
			*out =(int32_t)(LIS302DL_SENSITIVITY_2_3G *  (int8_t)buffer[2*i]);
			out++;
		}
		break;
	case 0x20:
		for(i=0; i<0x03; i++)
		{
			*out =(int32_t)(LIS302DL_SENSITIVITY_9_2G * (int8_t)buffer[2*i]);
			out++;
		}
		break;
	
	default:
		break;
	}
}


void lis302dl_read(struct LIS302DL *handle, uint8_t *buffer, uint8_t read_addr, uint16_t read_num)
{
	if (read_num > 0x01)
	{
		read_addr |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
	}
	else
	{
		read_addr |= (uint8_t)READWRITE_CMD;
	}

	handle->read(handle->spi, buffer, read_addr, read_num);
}


void lis302dl_write(struct LIS302DL *handle, uint8_t *buffer, uint8_t write_addr, uint16_t write_num)
{
	if (write_num > 0x01)
	{
		write_addr |= (uint8_t)MULTIPLEBYTE_CMD;
	}

	handle->write(handle->spi, buffer, write_addr, write_num);
}

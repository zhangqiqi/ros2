#ifndef __LIS302DL_H__
#define __LIS302DL_H__

#include <stdlib.h>
#include <stdint.h>

#include "stm32f4_discovery_lis302dl.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef void (*LIS302DL_READ_IF)(void *spi, uint8_t *buffer, uint8_t read_addr, uint16_t read_num);

typedef void (*LIS302DL_WRITE_IF)(void *spi, uint8_t *buffer, uint8_t write_addr, uint16_t write_num);

struct LIS302DL;

struct LIS302DL *lis302dl_init(void *spi, LIS302DL_READ_IF read_if, LIS302DL_WRITE_IF write_if);

void lis302dl_interrupt_config(struct LIS302DL *handle, uint8_t latch_request, uint8_t single_click_axes, uint8_t double_click_axes);

void lis302dl_highpass_filter_config(struct LIS302DL *handle, uint8_t data_selection, uint8_t cutoff_freq, uint8_t interrupt);

void lis302dl_lowpower_cmd(struct LIS302DL *handle, uint8_t lowpower_mode);

void lis302dl_fullscale_cmd(struct LIS302DL *handle, uint8_t fullscale_value);

void lis302dl_datarate_cmd(struct LIS302DL *handle, uint8_t datarate_value);

void lis302_reboot_cmd(struct LIS302DL *handle);

void lis302dl_read_acc(struct LIS302DL *handle, int32_t *out);

void lis302dl_read(struct LIS302DL *handle, uint8_t *buffer, uint8_t read_addr, uint16_t read_num);

void lis302dl_write(struct LIS302DL *handle, uint8_t *buffer, uint8_t write_addr, uint16_t write_num);

#ifdef __cplusplus
}
#endif


#endif


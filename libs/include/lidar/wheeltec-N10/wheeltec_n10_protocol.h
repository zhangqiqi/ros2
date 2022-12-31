#ifndef __DEVICE_LIDAR_WHEELTEC_N10_H__
#define __DEVICE_LIDAR_WHEELTEC_N10_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(1)

/**
 * @brief 镭神N10激光雷达数据单点数据结构
 */
struct WHEELTEC_N10_POINT {
	uint8_t distance_h;      /**< 距离数据高字节 单位 mm */
	uint8_t distance_l;      /**< 距离数据低字节 单位 mm */
	uint8_t peak;      /**< 强度数据 */
};


/**
 * @brief 镭神N10激光雷达点云协议帧
 */
struct WHEELTEC_N10_FRAME {
	uint16_t head;      /**< 点云帧头 0x5aa5 */
	uint8_t length;      /**< 数据长度 */
	uint8_t speed_h;      /**< 转速信息高字节 */
	uint8_t speed_l;      /**< 转速信息低字节 */
	uint8_t start_angle_h;      /**< 帧数据起始角度高字节 */
	uint8_t start_angle_l;      /**< 帧数据起始角度低字节 */
	struct WHEELTEC_N10_POINT points[18];      /**< 根据协议文档，每帧数据包含18个点信息 */
	uint8_t stop_angle_h;      /**< 帧数据结束角度高字节 */
	uint8_t stop_angle_l;      /**< 帧数据结束角度低字节 */
	uint8_t crc;      /**< 从帧头开始到crc前一个字节的和校验 */
};

#pragma pack()

int32_t wheeltec_n10_frame_unpack(uint8_t *data, int32_t len, struct WHEELTEC_N10_FRAME **frame);


#ifdef __cplusplus
}
#endif

#endif //__DEVICE_LIDAR_WHEELTEC_N10_H__

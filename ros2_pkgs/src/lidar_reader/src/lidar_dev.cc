#include "lidar_dev.h"
#include <cerrno>
#include <cstring>
#include <iterator>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/detail/laser_scan__struct.hpp>
#include <string>
#include <fcntl.h>
#include <strings.h>
#include <termios.h>



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
    struct WHEELTEC_N10_POINT points[16];      /**< 根据协议文档，每帧数据包含16个点信息 */
	uint8_t stop_angle_h;      /**< 帧数据结束角度高字节 */
	uint8_t stop_angle_l;      /**< 帧数据结束角度低字节 */
	uint8_t crc;      /**< 从帧头开始到crc前一个字节的和校验 */
};

#pragma pack()

/**
 * @brief n10帧数据和校验算法
 * @param data 进行校验的数据缓存区地址
 * @param len 进行校验的数据长度
 * @return uint8_t 计算得到的和校验值
 */
static uint8_t wheeltec_n10_checksum(uint8_t *data, int32_t len)
{
	uint8_t checksum = 0;

	while (len-- > 0)
	{
		checksum += data[len];
	}

	return checksum;
}


/**
 * @brief 按顺序从输入缓存区中，检索并校验点云帧数据
 * @param data 数据输入缓存区地址
 * @param len 数据输入缓存区数据长度
 * @param frame 解析校验通过的帧数据首地址写入地址
 * @return int32_t 本次操作完成后，输入缓存区的失效数据长度
 */
int32_t wheeltec_n10_frame_unpack(uint8_t *data, int32_t len, struct WHEELTEC_N10_FRAME **frame)
{
	struct WHEELTEC_N10_FRAME *_frame = NULL;
	*frame = NULL;

	int i = 0;
	for (i = 0; i < len; i++)
	{
		_frame = (struct WHEELTEC_N10_FRAME *)(data + i);
		if (0x5aa5 != _frame->head)
		{
			continue;
		}

		if ((len - i) < sizeof(struct WHEELTEC_N10_FRAME))
		{
			break;
		}

		if (wheeltec_n10_checksum(data + i, sizeof(struct WHEELTEC_N10_FRAME) - 1) != _frame->crc)
		{
			continue;
		}

		*frame = _frame;
		i = i + sizeof(struct WHEELTEC_N10_FRAME);
		break;
	}

	return i;
}

#ifdef __cplusplus
}
#endif



LidarDev::LidarDev(rclcpp::Node &parent, std::string port, int speed)
		: node(parent), dev_sp(port, speed) 
{
	if (0 != dev_sp.open())	
	{
		RCLCPP_INFO(node.get_logger(), "lidar device open failed, err: %s", dev_sp.get_errstring().c_str());
		return;
	}
	
	RCLCPP_INFO(node.get_logger(), "lidar device construct success");
}


std::shared_ptr<sensor_msgs::msg::LaserScan> LidarDev::get_next_msgs(void)
{

	if (lidar_msgs.empty())
	{
		return nullptr;
	}
	auto ret = lidar_msgs.front();
	lidar_msgs.pop_front();
	return ret;
}

void LidarDev::push_lidar_msgs(const struct WHEELTEC_N10_FRAME *frame)
{
	auto _new_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
	_new_msg->header.stamp = node.now();
	_new_msg->header.frame_id = "lidar_link";

	float start_angle = (frame->start_angle_h << 8) + frame->start_angle_l;
	float stop_angle = (frame->stop_angle_h << 8) + frame->stop_angle_l;
	float speed = (frame->speed_h << 8) + frame->speed_l;
	
	if (stop_angle < start_angle)
	{
		stop_angle = 36000;
	}

	int cnt = sizeof(frame->points) / sizeof(frame->points[0]);
	for (int i = 0; i < cnt; ++i)	
	{
		float ranges = (frame->points[i].distance_h << 8) + frame->points[i].distance_l;
		float intensities = frame->points[i].peak;

		_new_msg->ranges.push_back(ranges / 1000);
		_new_msg->intensities.push_back(intensities / 1000);
	}

	_new_msg->angle_min = start_angle * std::numbers::pi / 36000;
	_new_msg->angle_max = stop_angle * std::numbers::pi / 36000;
	_new_msg->angle_increment = 0.8 * std::numbers::pi / 360;
	_new_msg->time_increment = speed / (32 *  1000 * 1000);
	_new_msg->range_max = 12;
	_new_msg->range_min = 0.015;
	
//	RCLCPP_INFO(node.get_logger(), "new lidar msg: angle(%f, %f), speed(%f)", 
//		_new_msg->angle_min, _new_msg->angle_max, speed);

	lidar_msgs.push_back(_new_msg);
}

void LidarDev::exec()
{
	uint8_t data[1024] = {0};
	int32_t size = 0;

	size = dev_sp.read(data, sizeof(data));
	if (size < 0)
	{
		RCLCPP_INFO(node.get_logger(), "get lidar data failed, err: %s", dev_sp.get_errstring().c_str()); 
		return;
	}
	buffer.insert(buffer.end(), std::begin(data), std::begin(data) + size);	

	do
	{
		struct WHEELTEC_N10_FRAME *frame = nullptr;
		size = wheeltec_n10_frame_unpack(buffer.data(), buffer.size(), &frame);	
		if (nullptr != frame)
		{
			push_lidar_msgs(frame);
		}
		if (size > 0)
		{
			RCLCPP_DEBUG(node.get_logger(), "erase lidar buffer size: %d", size);
			buffer.erase(buffer.begin(), buffer.begin() + size);
		}
		if (buffer.size() < sizeof(struct WHEELTEC_N10_FRAME))
		{
			break;
		}
	} while (true);
}


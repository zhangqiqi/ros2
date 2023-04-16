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
#include <vector>



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

class LaserScanPoints
{
public:
	LaserScanPoints(rclcpp::Node &parent)
		: node(parent)
	{
	
	}
	
	bool append(const WHEELTEC_N10_FRAME &frame, double cur_stamp)
	{
		double start_angle = (frame.start_angle_h << 8) + frame.start_angle_l;
		double stop_angle = (frame.stop_angle_h << 8) + frame.stop_angle_l;
		
		double angle_step = (stop_angle > start_angle) ?
				(stop_angle - start_angle) / 15 : 
				(stop_angle + 36000 - start_angle) / 15;

		double cur_angle = start_angle;
		double stamp_step = (frame.speed_h << 8) + frame.speed_l;
		stamp_step = stamp_step / (2 * 16 * 1000 * 1000);		
		
		for (auto point = std::begin(frame.points); point != std::end(frame.points); ++point)
		{
			double _distance = double((point->distance_h << 8) + point->distance_l) / 1000;
			double _peak = (double)point->peak / 1000;
			distance.push_back(_distance);	
			peak.push_back(_peak);
			stamp.push_back(cur_stamp);
			if (cur_angle > 36000)
			{
				angle.push_back(cur_angle - 36000);
			}
			else
			{
				angle.push_back(cur_angle);
			}
			cur_angle += angle_step;
			cur_stamp += stamp_step;
		}
		return cur_angle > 36000;
	}
	
	void get_laserscan_msg(std::shared_ptr<sensor_msgs::msg::LaserScan> msg)
	{
		decltype(angle.size()) i = 0;
		for (i = 0; i < angle.size(); ++i)			
		{
			msg->ranges.push_back(distance[i]);
			msg->intensities.push_back(peak[i]);
			if ((i + 1) >= angle.size())
			{
				break;
			}
			if (angle[i] > angle[i + 1])
			{
		//		RCLCPP_INFO(node.get_logger(), "angle[i] = %f, angle[i + 1] = %f", angle[i], angle[i + 1]);	
				break;
			}
		}
		
		msg->angle_min = angle[0] * 2 * std::numbers::pi / 36000;	
		msg->angle_max = angle[i] * 2 * std::numbers::pi / 36000;

		msg->angle_increment = (msg->angle_max - msg->angle_min) / (msg->ranges.size() - 1);
		msg->time_increment = (stamp[i] - stamp[0]) / (msg->ranges.size() - 1);

		distance.erase(distance.begin(), distance.begin() + i + 1);	
		peak.erase(peak.begin(), peak.begin() + i + 1);
		angle.erase(angle.begin(), angle.begin() + i + 1);
		stamp.erase(stamp.begin(), stamp.begin() + i + 1);
		
		RCLCPP_INFO(node.get_logger(), "get new laser msg, start stamp(%lf) angle(%lf), stop stamp(%lf) angle(%lf)", stamp[0], msg->angle_min, stamp[i], msg->angle_max);
	}

private:
	rclcpp::Node &node;
	std::vector<double> distance;
	std::vector<double> peak;
	std::vector<double> angle;
	std::vector<double> stamp;	

};

LidarDev::LidarDev(rclcpp::Node &parent, std::string port, int speed)
		: node(parent), dev_sp(port, speed) 
{
	if (0 != dev_sp.open())	
	{
		RCLCPP_INFO(node.get_logger(), "lidar device open failed, err: %s", dev_sp.get_errstring().c_str());
		return;
	}
	points = std::make_shared<LaserScanPoints>(parent);	
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

void LidarDev::push_lidar_msgs()
{

	auto _new_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
//	_new_msg->header.stamp = node.now();
	_new_msg->header.frame_id = "lidar_link";
	
	points->get_laserscan_msg(_new_msg);

	_new_msg->range_max = 12;
	_new_msg->range_min = 0.015;
	
	//RCLCPP_INFO(node.get_logger(), "new lidar msg: angle(%f, %f), points num(%ld)", 
	//	_new_msg->angle_min, _new_msg->angle_max, _new_msg->ranges.size());

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
			if (true == points->append(*frame, node.now().seconds()))
			{
				push_lidar_msgs();
			}
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


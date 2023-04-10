#ifndef __LIDAR_DEV_H__
#define __LIDAR_DEV_H__

#include <memory>
#include <sensor_msgs/msg/detail/laser_scan__struct.hpp>
#include <string>
#include <vector>
#include <list>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "common/SerialPort.h"



struct WHEELTEC_N10_FRAME;

class LidarDev
{
public:
	LidarDev(rclcpp::Node &parent, std::string port, int speed);

	std::shared_ptr<sensor_msgs::msg::LaserScan> get_next_msgs(void);

	void exec();
private:
	void push_lidar_msgs(const struct WHEELTEC_N10_FRAME *frame);

	rclcpp::Node &node;
	Common::SerialPort dev_sp;
	
	std::vector<uint8_t> buffer;	
	std::list<std::shared_ptr<sensor_msgs::msg::LaserScan>> lidar_msgs;
};


#endif


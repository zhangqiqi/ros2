#ifndef __LIDAR_DEV_H__
#define __LIDAR_DEV_H__

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "common/SerialPort.h"

class LidarDev
{
public:
	LidarDev(rclcpp::Node &parent, std::string port, int speed);

	void exec();
private:
	int32_t lidar_setopt();
	
	rclcpp::Node &node;
	Common::SerialPort dev_sp;
};


#endif


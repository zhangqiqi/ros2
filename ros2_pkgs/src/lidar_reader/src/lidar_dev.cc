#include "lidar_dev.h"
#include <cerrno>
#include <cstring>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <string>
#include <fcntl.h>
#include <strings.h>
#include <termios.h>


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


void LidarDev::exec()
{
	uint8_t buffer[1024] = {0};
	int32_t size = 0;

	size = dev_sp.read(buffer, size);	
	if (size > 0)
	{
		RCLCPP_INFO(node.get_logger(), "get new lidar data size: %d", size);
	}

}


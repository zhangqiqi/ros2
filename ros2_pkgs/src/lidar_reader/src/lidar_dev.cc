#include "lidar_dev.h"
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <string>


LidarDev::LidarDev(rclcpp::Node &parent, std::string port, int speed)
		: node(parent), port(port), speed(speed)
{
	this->node.get_logger();
	RCLCPP_INFO(this->node.get_logger(), "lidar device construct success, hello %d", 666);
}



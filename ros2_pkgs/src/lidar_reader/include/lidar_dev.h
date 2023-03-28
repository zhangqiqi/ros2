#ifndef __LIDAR_DEV_H__
#define __LIDAR_DEV_H__

#include <string>
#include "rclcpp/rclcpp.hpp"


class LidarDev
{
public:
	LidarDev(rclcpp::Node &parent, std::string port, int speed);

private:
	rclcpp::Node &node;
	std::string port;
	int speed;
};


#endif


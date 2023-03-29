#ifndef __LIDAR_DEV_H__
#define __LIDAR_DEV_H__

#include <string>
#include "rclcpp/rclcpp.hpp"


class LidarDev
{
public:
	LidarDev(rclcpp::Node &parent, std::string port, int speed);
	int32_t lidar_open();
	void lidar_close();

	void exec();
private:
	int32_t lidar_setopt();

	rclcpp::Node &node;
	std::string port;
	int speed;

	int fd;
};


#endif


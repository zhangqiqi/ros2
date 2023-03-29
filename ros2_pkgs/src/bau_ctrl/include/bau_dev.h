#ifndef __BAU_DEV_H__
#define __BAU_DEV_H__


#include <rclcpp/node.hpp>
#include <string>
#include "rclcpp/rclcpp.hpp"


class BauDev
{
public:
	BauDev(rclcpp::Node &parent, std::string port, int speed);
	int32_t bau_open();
	void bau_close();
	
	void exec();
private:
	int32_t bau_setopt();

	rclcpp::Node &node;
	std::string port;
	int speed;
	
	int fd;
};

#endif


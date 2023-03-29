#include "bau_dev.h"
#include <cerrno>
#include <cstring>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <string>
#include <fcntl.h>
#include <strings.h>
#include <termios.h>

BauDev::BauDev(rclcpp::Node &parent, std::string port, int speed)
		: node(parent), port(port), speed(speed) 
{
	RCLCPP_INFO(node.get_logger(), "bau device construct success");
}


int32_t BauDev::bau_open()
{
	fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd < 0)
	{
		RCLCPP_INFO(node.get_logger(), "open bau dev port %s failed, err: %d, %s", port.c_str(), errno, strerror(errno));
		return -1;
	}

	int flags = 0;
	flags = fcntl(fd, F_GETFL, 0);
	flags &= ~O_NONBLOCK;

	if (fcntl(fd, F_SETFL, flags) < 0)
	{
		RCLCPP_INFO(node.get_logger(), "fcntl bau dev failed, err: %s", strerror(errno));
		return -2;
	}
	
	return bau_setopt();
}


void BauDev::bau_close()
{
	close(fd);
}


int32_t BauDev::bau_setopt()
{
	struct termios new_tio;
	struct termios old_tio;
	
	bzero(&new_tio, sizeof(new_tio));
	bzero(&old_tio, sizeof(old_tio));

	if (0 != tcgetattr(fd, &old_tio))
	{
		RCLCPP_INFO(node.get_logger(), "%s", strerror(errno));
		return -1;
	}

	new_tio.c_cflag |= CLOCAL | CREAD;
	cfsetspeed(&new_tio, B115200);
	
	new_tio.c_cflag |= CS8;
	new_tio.c_cflag &= ~PARENB;
	new_tio.c_cflag &= ~INPCK;
	new_tio.c_cflag &= ~CSTOPB;
	new_tio.c_cflag &= ~CRTSCTS;

	new_tio.c_cc[VTIME] = 10;
	new_tio.c_cc[VMIN] = 0;

	tcflush(fd, TCIOFLUSH);	

	if (0 != tcsetattr(fd, TCSANOW, &new_tio))
	{
		RCLCPP_INFO(node.get_logger(), "%s", strerror(errno));
		return -2;
	}
	return 0;
}


void BauDev::exec()
{
	int32_t exec_code = 0;
	do
	{
		if (fd < 0)	
		{
			exec_code = -1;
			break;
		}

		uint8_t buffer[1024] = {0};
		int32_t len = read(fd, buffer, sizeof(buffer));
		if (len < 0)
		{
			RCLCPP_INFO(node.get_logger(), "bau read failed, err: %s", strerror(errno));
			exec_code = -2;
			break;
		}
		
		RCLCPP_INFO(node.get_logger(), "bau get new raw data, len: %d", len);
		
	} while (false);	
	
	if (exec_code < 0)
	{
		RCLCPP_INFO(node.get_logger(), "bau exec failed, exec code: %d", exec_code);
	}
}



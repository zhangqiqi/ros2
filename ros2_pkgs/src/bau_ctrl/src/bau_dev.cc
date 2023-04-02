#include "bau_dev.h"
#include "ssnp/ssnp.h"
#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <string>
#include <fcntl.h>
#include <strings.h>
#include <termios.h>

#include "ssnp/ssnp_buffer.h"
#include "ssnp/ssnp_msgs.h"
#include "ssnp/ssnp_shell.h"
#include "ssnp/ssnp_msgs_def.h"

#ifdef __cplusplus
extern "C" {
#endif


int32_t ssnp_recv_cb(void *read_handle, struct SSNP_BUFFER *recv_buf)
{
	BauDev *dev = static_cast<BauDev *>(read_handle);	

	uint8_t buffer[1024] = {0};
	int32_t len = read(dev->fd, buffer, sizeof(buffer));
	if (len < 0)
	{
		RCLCPP_INFO(dev->node.get_logger(), "bau read failed, err: %s", strerror(errno));
	}
	else
	{
		ssnp_buffer_write(recv_buf, buffer, len);

		RCLCPP_INFO(dev->node.get_logger(), "bau get new raw data, len: %d", len);
	}	
	
	return 0;
}

int32_t ssnp_trans_cb(void *write_handle, struct SSNP_BUFFER *trans_buf)
{
	BauDev *dev = static_cast<BauDev *>(write_handle);
	
	uint8_t *buffer = NULL;

	int32_t len = ssnp_buffer_copyout_ptr(trans_buf, &buffer, 1024);
	if (len > 0 && NULL != buffer)
	{
		write(dev->fd, buffer, len);
	}

	return 0;
}


int32_t ssnp_proc_shell_res_msg(void *cb_handle, struct SSNP *ssnp, struct SSNP_FRAME *msg)
{
	BauDev *dev = static_cast<BauDev *>(cb_handle);

	struct SMT_SHELL_RES_MSG *_sub_msg = (struct SMT_SHELL_RES_MSG *)msg->payload;

	RCLCPP_INFO(dev->node.get_logger(), "ssnp shell>>%s", _sub_msg->res_str);

	return 0;
}


void ssnp_log_print_if(void *log_handle, char *fmt, ...)
{
	BauDev *dev = static_cast<BauDev *>(log_handle);
	char log_str[1024] = {0};

	va_list args;
	va_start(args, fmt);
	vsnprintf(log_str, sizeof(log_str), fmt, args);
	va_end(args);

	RCLCPP_INFO(dev->node.get_logger(), "%s", log_str);	
}

#ifdef __cplusplus
}
#endif


BauDev::BauDev(rclcpp::Node &parent, std::string port, int speed)
		: node(parent), port(port), speed(speed) 
{
	ssnp_shell_init();	
	ssnp_log_print_setup(this, ssnp_log_print_if);
	ssnp = ssnp_create();
	
	ssnp_recv_if_setup(ssnp, this, ssnp_recv_cb);
	ssnp_trans_if_setup(ssnp, this, ssnp_trans_cb);

	if (0 != ssnp_msgs_listener_setup(ssnp, SMT_SHELL_RES, ssnp_proc_shell_res_msg, this))
	{
		RCLCPP_INFO(node.get_logger(), "shell res msg proc setup failed");
	}

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
	
	cfsetspeed(&new_tio, B115200);
	
	new_tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | ICRNL | IXON);
	new_tio.c_oflag &= ~OPOST;
	new_tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	new_tio.c_cflag &= ~(CSIZE | PARENB);
	new_tio.c_cflag |= CS8;	

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

		exec_code = ssnp_exec(ssnp);	
	} while (false);	
	
	if (exec_code < 0)
	{
		RCLCPP_INFO(node.get_logger(), "bau exec failed, exec code: %d", exec_code);
	}
}


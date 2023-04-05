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
#include <numbers> 

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
		int32_t ret = write(dev->fd, buffer, len);
		RCLCPP_INFO(dev->node.get_logger(), "bau send raw data, len: %d", ret);
		if (ret < 0)
		{
			RCLCPP_INFO(dev->node.get_logger(), "bau send failed, err: %s", strerror(errno));
		}

		ssnp_buffer_drain(trans_buf, len);
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


int32_t ssnp_proc_wheel_motor_data(void *cb_handle, struct SSNP *ssnp, struct SSNP_FRAME *msg)
{
	BauDev *dev = static_cast<BauDev *>(cb_handle);
	struct SMT_WHEEL_MOTOR_DATA_PUSH_MSG *_sub_msg = (struct SMT_WHEEL_MOTOR_DATA_PUSH_MSG *)msg->payload;
	
	RCLCPP_INFO(dev->node.get_logger(), "left: freq %d, target %d, sample %d; right: freq %d, target %d, sample %d",
		_sub_msg->left_motor_data.freq, _sub_msg->left_motor_data.target_count, _sub_msg->left_motor_data.sample_count,
		_sub_msg->right_motor_data.freq, _sub_msg->right_motor_data.target_count, _sub_msg->right_motor_data.sample_count		
	);

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


BauDev::BauDev(rclcpp::Node &parent, std::string port, int baudrate)
		: node(parent), port(port), baudrate(baudrate) 
{
	ssnp_shell_init();	
	ssnp_log_print_setup(this, ssnp_log_print_if);
	// ssnp_set_log_level(SLT_DEBUG);
	ssnp = ssnp_create();
	
	ssnp_recv_if_setup(ssnp, this, ssnp_recv_cb);
	ssnp_trans_if_setup(ssnp, this, ssnp_trans_cb);

	if (0 != ssnp_msgs_listener_setup(ssnp, SMT_SHELL_RES, ssnp_proc_shell_res_msg, this))
	{
		RCLCPP_INFO(node.get_logger(), "shell res msg proc setup failed");
	}
	if (0 != ssnp_msgs_listener_setup(ssnp, SMT_WHEEL_MOTOR_DATA_PUSH, ssnp_proc_wheel_motor_data, this))
	{
		RCLCPP_INFO(node.get_logger(), "wheel motor data proc setup failed");
	}

	RCLCPP_INFO(node.get_logger(), "bau device construct success");
}


void BauDev::set_wheel_params(float rpm, float ratio, float scrl, float radius)
{
	this->rpm = rpm;
	this->ratio = ratio;
	this->scrl = scrl;
	this->radius = radius;
}


int32_t BauDev::speed_to_encoder_count(float speed)
{
	return (speed / (2 * radius * std::numbers::pi)) * scrl;
}


float BauDev::encoder_count_to_speed(int32_t count)
{
	return (count / scrl) * 2 * radius * std::numbers::pi;
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


int32_t BauDev::set_speed(float linear, float angular)
{
	int32_t freq = 100;
	int32_t converted_left_count = speed_to_encoder_count(linear);	
	int32_t converted_right_count;

	float converted_speed = encoder_count_to_speed(converted_left_count);
	RCLCPP_INFO(node.get_logger(), "src linear value: %f, convert liner speed: %f to cnt %d", linear, converted_speed, converted_left_count); 

	struct SMT_WHEEL_MOTOR_CTRL_MSG msg = {
		.freq = freq,
		.left_motor_count = converted_left_count,
		.right_motor_count = converted_right_count,
	};
	
	return ssnp_send_msg(ssnp, SMT_WHEEL_MOTOR_CTRL, (uint8_t *)&msg, sizeof(msg));	
}


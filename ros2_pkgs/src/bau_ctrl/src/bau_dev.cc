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
	int32_t len = dev->dev_sp.read(buffer, sizeof(buffer));
	if (len < 0)
	{
		RCLCPP_DEBUG(dev->node.get_logger(), "bau read failed, err: %s", strerror(errno));
	}
	else
	{
		ssnp_buffer_write(recv_buf, buffer, len);

		RCLCPP_DEBUG(dev->node.get_logger(), "bau get new raw data, len: %d", len);
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
		int32_t ret = dev->dev_sp.write(buffer, len);
		RCLCPP_DEBUG(dev->node.get_logger(), "bau send raw data, len: %d", ret);
		if (ret < 0)
		{
			RCLCPP_INFO(dev->node.get_logger(), "bau send failed, err: %s", strerror(errno));
		}

		ssnp_buffer_drain(trans_buf, len);
	}

	return 0;
}


int32_t ssnp_proc_shell_res_msg(void *cb_handle, [[maybe_unused]] struct SSNP *ssnp, struct SSNP_FRAME *msg)
{
	BauDev *dev = static_cast<BauDev *>(cb_handle);
	struct SMT_SHELL_RES_MSG *_sub_msg = (struct SMT_SHELL_RES_MSG *)msg->payload;

	RCLCPP_INFO(dev->node.get_logger(), "ssnp shell>>%s", _sub_msg->res_str);

	return 0;
}


int32_t ssnp_proc_wheel_motor_data(void *cb_handle, [[maybe_unused]] struct SSNP *ssnp, struct SSNP_FRAME *msg)
{
	BauDev *dev = static_cast<BauDev *>(cb_handle);
	struct SMT_WHEEL_MOTOR_DATA_PUSH_MSG *_sub_msg = (struct SMT_WHEEL_MOTOR_DATA_PUSH_MSG *)msg->payload;
	
	RCLCPP_DEBUG(dev->node.get_logger(), "left: freq %d, target %d, sample %d; right: freq %d, target %d, sample %d",
		_sub_msg->left_motor_data.freq, _sub_msg->left_motor_data.target_count, _sub_msg->left_motor_data.sample_count,
		_sub_msg->right_motor_data.freq, _sub_msg->right_motor_data.target_count, _sub_msg->right_motor_data.sample_count		
	);

	double left_speed = dev->encoder_count_to_speed(_sub_msg->left_motor_data.sample_count * _sub_msg->left_motor_data.freq);
	double right_speed = dev->encoder_count_to_speed(_sub_msg->right_motor_data.sample_count * _sub_msg->right_motor_data.freq);

	dev->wheel_speed_to_car_speed(right_speed, left_speed, dev->cur_linear, dev->cur_angular);

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
		: node(parent), dev_sp(port, baudrate), cur_linear(0.0), cur_angular(0.0), cur_linear_target(0.), cur_angular_target(0.)
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

	if (0 != dev_sp.open())
	{
		RCLCPP_INFO(node.get_logger(), "%s", dev_sp.get_errstring().c_str());
	}

	RCLCPP_INFO(node.get_logger(), "bau device construct success");
}


void BauDev::set_wheel_params(double rpm, double ratio, double scrl, double radius, double wheel_spacing)
{
	this->rpm = rpm;
	this->ratio = ratio;
	this->scrl = scrl;
	this->radius = radius;
	this->wheel_spacing = wheel_spacing;
}


int32_t BauDev::speed_to_encoder_count(double speed)
{
	return (speed / (2 * radius * std::numbers::pi)) * scrl;
}


double BauDev::encoder_count_to_speed(int32_t count)
{
	return (count / scrl) * 2 * radius * std::numbers::pi;
}


void BauDev::exec()
{
	int32_t exec_code = ssnp_exec(ssnp);	
	
	if (exec_code < 0)
	{
		RCLCPP_DEBUG(node.get_logger(), "bau exec failed, exec code: %d", exec_code);
	}
}


void BauDev::car_speed_to_wheel_speed(double linear, double angular, double &Vr, double &Vl)
{
	Vr = linear + angular * wheel_spacing / 2;	
	Vl = linear - angular * wheel_spacing / 2; 
}


int32_t BauDev::get_speed(double &linear, double &angular)
{
	linear = cur_linear / 1000;
	angular = cur_angular;
//	linear = cur_linear_target / 1000;
//	angular = cur_angular_target;

	return 0;
}


void BauDev::wheel_speed_to_car_speed(const double &Vr, const double &Vl, double &linear, double &angular)
{
	linear = (Vr + Vl) / 2;
	angular = (Vr - Vl) / wheel_spacing;

	RCLCPP_DEBUG(node.get_logger(), "runtime speed: linear %lf, angular %lf", linear, angular);
}


int32_t BauDev::set_speed(double linear, double angular)
{
	int32_t freq = 100;
	double Vr, Vl;

	linear *= 1000;

	cur_linear_target = linear;
	cur_angular_target = angular;

	car_speed_to_wheel_speed(linear, angular, Vr, Vl);

	int32_t converted_left_count = speed_to_encoder_count(Vl);	
	int32_t converted_right_count= speed_to_encoder_count(Vr);

	RCLCPP_INFO(node.get_logger(), "dst speed linear(%f) angular(%f), convert to bau ctrl left (%d) right (%d)", linear, angular, converted_left_count, converted_right_count);

	struct SMT_WHEEL_MOTOR_CTRL_MSG msg = {
		.freq = freq,
		.left_motor_count = converted_left_count / freq,
		.right_motor_count = converted_right_count / freq,
	};

	RCLCPP_INFO(node.get_logger(), "send left motor count %d, right motor count %d", msg.left_motor_count, msg.right_motor_count);
	
	return ssnp_send_msg(ssnp, SMT_WHEEL_MOTOR_CTRL, (uint8_t *)&msg, sizeof(msg));	
}


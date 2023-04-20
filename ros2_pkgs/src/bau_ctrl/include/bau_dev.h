#ifndef __BAU_DEV_H__
#define __BAU_DEV_H__


#include <rclcpp/node.hpp>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "ssnp/ssnp.h"
#include "common/SerialPort.h"

#ifdef __cplusplus
extern "C" {
#endif
int32_t ssnp_recv_cb(void *read_handle, struct SSNP_BUFFER *recv_buf);

int32_t ssnp_trans_cb(void *write_handle, struct SSNP_BUFFER *trans_buf);

int32_t ssnp_proc_shell_res_msg(void *cb_handle, struct SSNP *ssnp, struct SSNP_FRAME *msg);

int32_t ssnp_proc_wheel_motor_data(void *cb_handle, struct SSNP *ssnp, struct SSNP_FRAME *mgs);

void ssnp_log_print_if(void *log_handle, char *fmt, ...);
#ifdef __cplusplus
}
#endif


class BauDev
{
public:
	BauDev(rclcpp::Node &parent, std::string port, int baudrate);

	void set_wheel_params(double rpm, double ratio, double scrl, double radius, double wheel_spacing);

	int32_t set_speed(double linear, double angular);

	int32_t get_speed(double &linear, double &angular);

	void exec();
private:
	void car_speed_to_wheel_speed(double linear, double angular, double &Vr, double &Vl);
	void wheel_speed_to_car_speed(const double &Vr, const double &Vl, double &linear, double &angular);

	int32_t speed_to_encoder_count(double speed);
	double encoder_count_to_speed(int32_t count);

	friend int32_t ssnp_recv_cb(void *read_handle, struct SSNP_BUFFER *recv_buf);
	friend int32_t ssnp_trans_cb(void *write_handle, struct SSNP_BUFFER *trans_buf);
	friend int32_t ssnp_proc_shell_res_msg(void *cb_handle, struct SSNP *ssnp, struct SSNP_FRAME *msg);
	friend void ssnp_log_print_if(void *log_handle, char *fmt, ...);
	friend int32_t ssnp_proc_wheel_motor_data(void *cb_handle, struct SSNP *ssnp, struct SSNP_FRAME *mgs);

	rclcpp::Node &node;
	Common::SerialPort dev_sp;
	struct SSNP *ssnp;

	double rpm;
	double ratio;
	double scrl;
	double radius;
	double wheel_spacing;

	double cur_linear;
	double cur_angular;
};

#endif


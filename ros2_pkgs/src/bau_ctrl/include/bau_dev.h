#ifndef __BAU_DEV_H__
#define __BAU_DEV_H__


#include <rclcpp/node.hpp>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "ssnp/ssnp.h"

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
	BauDev(rclcpp::Node &parent, std::string port, int speed);
	int32_t bau_open();
	void bau_close();

	int32_t set_speed(float linear, float angular);

	void exec();
private:
	int32_t bau_setopt();
	
	friend int32_t ssnp_recv_cb(void *read_handle, struct SSNP_BUFFER *recv_buf);
	friend int32_t ssnp_trans_cb(void *write_handle, struct SSNP_BUFFER *trans_buf);
	friend int32_t ssnp_proc_shell_res_msg(void *cb_handle, struct SSNP *ssnp, struct SSNP_FRAME *msg);
	friend void ssnp_log_print_if(void *log_handle, char *fmt, ...);
	friend int32_t ssnp_proc_wheel_motor_data(void *cb_handle, struct SSNP *ssnp, struct SSNP_FRAME *mgs);

	struct SSNP *ssnp;

	rclcpp::Node &node;
	std::string port;
	int speed;
	
	int fd;
};

#endif


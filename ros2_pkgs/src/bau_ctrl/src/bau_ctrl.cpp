#include <chrono>
#include <memory>
#include <functional>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/subscription.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "bau_dev.h"


using namespace std::chrono_literals;

class BauCtrl : public rclcpp::Node
{
public:
	BauCtrl() : Node("bau_ctrl")
	{
		bau_dev = std::make_shared<BauDev>(*this, "/dev/ttyUSB0", 115200);
		if (nullptr == bau_dev)
		{
			RCLCPP_INFO(get_logger(), "create bau device failed");	
		}
		else
		{
			auto ret_code = bau_dev->bau_open();
			if (0 != ret_code)
			{
				RCLCPP_INFO(get_logger(), "open bau device failed");
			}
		}
		subscription_twist = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, 
			std::bind(&BauCtrl::subscription_twist_cb, this, std::placeholders::_1));

		callback_group_reentrant = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
		
		timer_ = this->create_wall_timer(
			500ms, std::bind(&BauCtrl::bau_exec_timer, this), callback_group_reentrant);	
	}

private:
	void subscription_twist_cb(const geometry_msgs::msg::Twist &cmd) const
	{
		RCLCPP_INFO(get_logger(), "get new twist cmd_vel");	
		(void)cmd;
	}
	void bau_exec_timer()
	{
		this->bau_dev->exec();
	}
	
	rclcpp::CallbackGroup::SharedPtr callback_group_reentrant ;
	rclcpp::TimerBase::SharedPtr timer_;
	std::shared_ptr<BauDev> bau_dev;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_twist;
};


int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<BauCtrl>();
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);
	executor.spin();

	rclcpp::shutdown();
  	return 0;
}

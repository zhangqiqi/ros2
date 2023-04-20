#include <chrono>
#include <geometry_msgs/msg/detail/twist_with_covariance_stamped__struct.hpp>
#include <memory>
#include <functional>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <tf2/tf2/LinearMath/Quaternion.h>
#include <tf2_ros/tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "bau_dev.h"


using namespace std::chrono_literals;

class BauCtrl : public rclcpp::Node
{
public:
	BauCtrl() : Node("bau_ctrl")
	{
		auto dev_name = this->declare_parameter("device_name", "/dev/ttyUSB0");
		auto dev_baudrate = this->declare_parameter("device_baudrate", 115200);
		auto wheel_motor_rpm = this->declare_parameter("wheel_motor_rpm", 150);
		auto wheel_motor_ratio = this->declare_parameter("wheel_motor_ratio", 75);
		auto wheel_encoder_scrl = this->declare_parameter("wheel_encoder_scrl", 8250);
		auto wheel_radius = this->declare_parameter("wheel_radius", 32.5);
		auto wheel_spacing = this->declare_parameter("wheel_spacing", 170);

		_twist_stamped_publisher = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("bau_ctrl/twist", 10);
		bau_dev = std::make_shared<BauDev>(*this, dev_name, dev_baudrate);
		if (nullptr == bau_dev)
		{
			RCLCPP_INFO(get_logger(), "create bau device failed");	
		}
		else
		{
			bau_dev->set_wheel_params(
				wheel_motor_rpm, 
				wheel_motor_ratio, 
				wheel_encoder_scrl, 
				wheel_radius,
				wheel_spacing
			);
		}
		subscription_twist = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, 
			std::bind(&BauCtrl::subscription_twist_cb, this, std::placeholders::_1));

		callback_group_reentrant = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
		
		timer_ = this->create_wall_timer(
			50ms, std::bind(&BauCtrl::bau_exec_timer, this), callback_group_reentrant);	
	}

private:
	void subscription_twist_cb(const geometry_msgs::msg::Twist &cmd) const
	{
		RCLCPP_INFO(get_logger(), "get new twist cmd_vel, linear: {x: %f y: %f z: %f}, angular: {x: %f y: %f z: %f}",
			cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.x, cmd.angular.y, cmd.angular.z
		);	
		
		bau_dev->set_speed(cmd.linear.x, cmd.angular.z);
	}
	void bau_exec_timer()
	{
		this->bau_dev->exec();

		double linear = 0;
		double angular = 0;
		bau_dev->get_speed(linear, angular);

		geometry_msgs::msg::TwistWithCovarianceStamped _twist;

		_twist.header.stamp = this->now();
		_twist.header.frame_id = "base_link";
		_twist.twist.twist.linear.x = linear;
		_twist.twist.twist.linear.y = 0;
		_twist.twist.twist.linear.z = 0;
		_twist.twist.twist.angular.x = 0;
		_twist.twist.twist.angular.y = 0;
		_twist.twist.twist.angular.z = angular;
		
		_twist_stamped_publisher->publish(_twist);
	}
	
	rclcpp::CallbackGroup::SharedPtr callback_group_reentrant ;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr _twist_stamped_publisher;
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

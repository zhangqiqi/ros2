#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "lidar_dev.h"


using namespace std::chrono_literals;

class LidarReader : public rclcpp::Node
{
public:
	LidarReader() : Node("lidar_reader")
	{
		lidar_dev = std::make_shared<LidarDev>(*this, std::string("/dev/ttyUSB0"), 230400);
		if (nullptr == lidar_dev)
		{
			RCLCPP_INFO(get_logger(), "create lidar device failed");	
		}
		else
		{
			auto ret_code = lidar_dev->lidar_open();
			if (0 != ret_code)
			{
				RCLCPP_INFO(get_logger(), "open lidar device failed");
			}
		}

		callback_group_reentrant = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

		timer_ = this->create_wall_timer(10ms, std::bind(&LidarReader::lidar_exec_timer, this));	
	}
private:
	void lidar_exec_timer()
	{
		this->lidar_dev->exec();
	}

	rclcpp::CallbackGroup::SharedPtr callback_group_reentrant ;
	rclcpp::TimerBase::SharedPtr timer_;
	std::shared_ptr<LidarDev> lidar_dev;
};



int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<LidarReader>();
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}

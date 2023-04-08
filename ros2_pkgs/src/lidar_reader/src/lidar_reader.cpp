#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/publisher.hpp>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
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

		callback_group_reentrant = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
		timer_ = this->create_wall_timer(2ms, std::bind(&LidarReader::lidar_exec_timer, this));	
		lidar_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("LaserScan", 10);
	}
private:
	void lidar_exec_timer()
	{
		this->lidar_dev->exec();

		do
		{
			auto msg = lidar_dev->get_next_msgs();
			if (nullptr == msg)
			{
				break;
			}
			RCLCPP_DEBUG(get_logger(), "pub new lidar msg: angle(%f, %f), angle_increment(%f), time_increment(%f)",
				msg->angle_min, msg->angle_max, msg->angle_increment, msg->time_increment
			);
			lidar_pub->publish(*msg);
		} while (true);
	}

	rclcpp::CallbackGroup::SharedPtr callback_group_reentrant ;
	rclcpp::TimerBase::SharedPtr timer_;
	std::shared_ptr<LidarDev> lidar_dev;
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub;
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

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
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
		this->lidar_dev = new LidarDev(*this, std::string("ttyUSB0"), 230400);
		timer_ = this->create_wall_timer(500ms, std::bind(&LidarReader::timer_callback, this));	
	}
private:
	void timer_callback()
	{
		RCLCPP_INFO(this->get_logger(), "lidar reader running...");
	}
	rclcpp::TimerBase::SharedPtr timer_;
	LidarDev *lidar_dev;
};



int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LidarReader>());
	rclcpp::shutdown();
	return 0;
}

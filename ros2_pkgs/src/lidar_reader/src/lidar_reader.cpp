#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class LidarReader : public rclcpp::Node
{
public:
	LidarReader() : Node("lidar_reader")
	{
		timer_ = this->create_wall_timer(500ms, std::bind(&LidarReader::timer_callback, this));	
	}
private:
	void timer_callback()
	{
		printf("lidar reader running...\r\n");
	}
	rclcpp::TimerBase::SharedPtr timer_;
};



int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LidarReader>());
	rclcpp::shutdown();
	return 0;
}

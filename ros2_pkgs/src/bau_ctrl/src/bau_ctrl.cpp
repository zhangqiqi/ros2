#include <chrono>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class BauCtrl : public rclcpp::Node
{
public:
	BauCtrl() : Node("bau_ctrl")
	{
		timer_ = this->create_wall_timer(
			500ms, std::bind(&BauCtrl::timer_callback, this));	
	}

private:
	void timer_callback()
	{
		std::cout << "bau ctrl running" << std::endl;	
	}

	rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<BauCtrl>());
	rclcpp::shutdown();
  	return 0;
}

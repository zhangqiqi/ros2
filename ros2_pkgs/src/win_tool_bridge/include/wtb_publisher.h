#ifndef __WIN_TOOL_BRIDGE_WTB_PUBLISHER__
#define __WIN_TOOL_BRIDGE_WTB_PUBLISHER__

#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;


class WinToolBridge : public rclcpp::Node
{
public:
	WinToolBridge();

private:
	void timer_cb();

	rclcpp::TimerBase::SharedPtr timer;

	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher;
};


#endif // __WIN_TOOL_BRIDGE_WTB_PUBLISHER__

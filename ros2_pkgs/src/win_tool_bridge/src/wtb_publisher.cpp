#include "wtb_publisher.h"


WinToolBridge::WinToolBridge() : Node("win_tool_bridge")
{
	timer = this->create_wall_timer(500ms, std::bind(&WinToolBridge::timer_cb, this));

	laser_scan_publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
}


void WinToolBridge::timer_cb()
{
	RCLCPP_INFO(this->get_logger(), "timer start...");
}

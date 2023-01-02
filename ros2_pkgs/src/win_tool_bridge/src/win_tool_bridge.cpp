#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "wtb_server.h"


using namespace std::chrono_literals;


class WinToolBridge : public rclcpp::Node
{
public:
    WinToolBridge() : Node("win_tool_bridge") {
        timer = this->create_wall_timer(500ms, std::bind(&WinToolBridge::timer_cb, this));
    }
private:
    void timer_cb() {
        RCLCPP_INFO(this->get_logger(), "timer start...");
    }

    rclcpp::TimerBase::SharedPtr timer;
};


int main(int argc, char ** argv)
{
    WtbServer server{WTB_PORT};

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WinToolBridge>());
    rclcpp::shutdown();

    return 0;
}

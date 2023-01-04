#include "wtb_server.h"
#include "wtb_publisher.h"


int main(int argc, char ** argv)
{
    WtbServer server{WTB_PORT};

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WinToolBridge>());
    rclcpp::shutdown();

    return 0;
}

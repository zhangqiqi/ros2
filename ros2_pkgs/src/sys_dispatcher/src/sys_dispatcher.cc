#include <chrono>
#include <functional>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2/LinearMath/Quaternion.h"

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


using namespace std::chrono_literals;

class SysDispatcher : public rclcpp::Node
{
public:
	SysDispatcher()
		: Node("sys_dispatcher")
	{
		_timer = this->create_wall_timer(1000ms, std::bind(&SysDispatcher::sys_dispatcher_exec_timer, this));
		_tf_publisher = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
		make_transforms();	
	}
private:
	void sys_dispatcher_exec_timer()
	{
		// laser_link_transforms();
	}
	

	void make_transforms()
	{
		geometry_msgs::msg::TransformStamped t;
	
		t.header.stamp = this->now();
		t.header.frame_id = "world";
		t.child_frame_id = "base_link";
		
		t.transform.translation.x = 0;
		t.transform.translation.y = 0;
		t.transform.translation.z = 0;
		tf2::Quaternion q;
		q.setRPY(0, 0, 0);
		t.transform.rotation.x = q.x();
		t.transform.rotation.y = q.y();
		t.transform.rotation.z = q.z();
		t.transform.rotation.w = q.w();
			
		_tf_publisher->sendTransform(t);


		t.header.stamp = this->now();
		t.header.frame_id = "base_link";
		t.child_frame_id = "lidar_link";

		t.transform.translation.x = 0;
		t.transform.translation.y = 0;
		t.transform.translation.z = 0;

		q.setRPY(0, 0, 0);
		t.transform.rotation.x = q.x();
		t.transform.rotation.y = q.y();
		t.transform.rotation.z = q.z();
		t.transform.rotation.w = q.w();

		_tf_publisher->sendTransform(t);
	}


	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _tf_publisher;
	rclcpp::TimerBase::SharedPtr _timer;	

};


int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);

	auto sys_dispatcher_node = std::make_shared<SysDispatcher>();

	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(sys_dispatcher_node);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}

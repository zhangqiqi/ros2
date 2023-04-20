/******************************************************************
基于串口通信的ROS小车基础控制器，功能如下：
1.实现ros控制数据通过固定的格式和串口通信，从而达到控制小车的移动
2.订阅了/cmd_vel主题，只要向该主题发布消息，就能实现对控制小车的移动
3.发布里程计主题/odm

串口通信说明：
1.写入串口
（1）内容：左右轮速度，单位为mm/s
（2）格式：１０字节,[右轮速度４字节][左轮速度４字节][结束符"\r\n"２字节]
2.读取串口
（1）内容：小车x,y坐标，方向角，线速度，角速度，单位依次为：mm,mm,rad,mm/s,rad/s
（2）格式：２１字节，[Ｘ坐标４字节][Ｙ坐标４字节][方向角４字节][线速度４字节][角速度４字节][结束符"\n"１字节]
*******************************************************************/

#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <cstdio>
#include <unistd.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

using namespace std::chrono_literals;

/*****************************************************************************/

//发送给下位机的左右轮速度，里程计的坐标和方向
union floatData //union的作用为实现char数组和float之间的转换
{
    float d;
    unsigned char data[4];
}right_speed_data,left_speed_data,position_x,position_y,oriention,vel_linear,vel_angular;

float ratio = 1000.0f ;   //转速转换比例，执行速度调整比例
float D = 0.2680859f ;    //两轮间距，单位是m
float linear_temp=0,angular_temp=0;//暂存的线速度和角速度

unsigned char data_terminal0=0x0d;  //“/r"字符
unsigned char data_terminal1=0x0a;  //“/n"字符

class CarUartNode : public rclcpp::Node
{
public:
    CarUartNode() : Node("car_uart_node") {
      subscription_ = this->create_subscription<geometry_msgs::Twist>(
      "cmd_vel", 10, std::bind(&CarUartNode::topic_callback, this, _1));
      tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
      timer_ = this->create_wall_timer(
        10ms, std::bind(&CarUartNode::timer_callback, this));
      odom_publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    }

private:
    void topic_callback(const geometry_msgs::Twist & cmd_input) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
 //     string port("/dev/ttyACM0");    //小车串口号
 //     unsigned long baud = 115200;    //小车串口波特率
 //     serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000)); //配置串口

      angular_temp = cmd_input.angular.z ;//获取/cmd_vel的角速度,rad/s
      linear_temp = cmd_input.linear.x ;//获取/cmd_vel的线速度.m/s

      //将转换好的小车速度分量为左右轮速度
      left_speed_data.d = linear_temp - 0.5f * angular_temp * D ;
      right_speed_data.d = linear_temp + 0.5f * angular_temp * D ;

      //存入数据到要发布的左右轮速度消息
      left_speed_data.d*=ratio;   //放大１０００倍，mm/s
      right_speed_data.d*=ratio;//放大１０００倍，mm/s

      for(int i=0;i<4;i++)    //将左右轮速度存入数组中发送给串口
      {
          speed_data[i]=right_speed_data.data[i];
          speed_data[i+4]=left_speed_data.data[i];
      }

      //在写入串口的左右轮速度数据后加入”/r/n“
      speed_data[8]=data_terminal0;
      speed_data[9]=data_terminal1;
      //写入数据到串口
      // todo write uart to motor
//      my_serial.write(speed_data,10);
    }

    void timer_callback()
    {
      string rec_buffer;  //串口数据接收变量
//      rec_buffer =my_serial.readline(25,"\n");    //获取串口发送来的数据
//      const char *receive_data=rec_buffer.data(); //保存串口发送来的数据
// todo get msg from uart car
      const char *receive_data=rec_buffer.data(); //保存串口发送来的数据

      rclcpp::Time now = this->get_clock()->now();
      geometry_msgs::msg::TransformStamped t;

      if(rec_buffer.length()==21) //串口接收的数据长度正确就处理并发布里程计数据消息
      {
        for(int i=0;i<4;i++)//提取X，Y坐标，方向，线速度，角速度
        {
            position_x.data[i]=receive_data[i];
            position_y.data[i]=receive_data[i+4];
            oriention.data[i]=receive_data[i+8];
            vel_linear.data[i]=receive_data[i+12];
            vel_angular.data[i]=receive_data[i+16];
        }
        //将X，Y坐标，线速度缩小1000倍
        position_x.d/=1000; //m
        position_y.d/=1000; //m
        vel_linear.d/=1000; //m/s

        t.header.stamp = now;
        t.header.frame_id = "world";
        t.child_frame_id = "base_controller";

        t.transform.translation.x = position_x.d;
        t.transform.translation.y = position_y.d;
        t.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(
          atof(0),
          atof(0),
          atof(oriention.d));
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_publisher_->sendTransform(t);

        nav_msgs::Odometry odom;//定义里程计对象
        //载入里程计时间戳
        odom.header.stamp = this->get_clock()->now();
        //里程计的父子坐标系
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";       
        //里程计位置数据：x,y,z,方向
        odom.pose.pose.position.x = position_x.d;     
        odom.pose.pose.position.y = position_y.d;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;       
        //载入线速度和角速度
        odom.twist.twist.linear.x = vel_linear.d;
        //odom.twist.twist.linear.y = odom_vy;
        odom.twist.twist.angular.z = vel_angular.d;    
        //发布里程计
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", "odom");
        odom_publisher_->publish(odom);
      }
    }

    rclcpp::Subscription<geometry_msgs::Twist>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;
    rclcpp::Publisher<nav_msgs::Odometry>::SharedPtr odom_publisher_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CarUartNode>());
  rclcpp::shutdown()
  
  return 0;
}

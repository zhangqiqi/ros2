## 常用测试指令

### 发布cmd_vel话题消息并设置线速度和角速度

	ros2 topic pub --rate 1 cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 2.0, z: 3.0}, angular:{x: 1.1, y: 1.2, z: 1.3}}"
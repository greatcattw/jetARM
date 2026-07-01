#!/bin/sh
ros2 topic pub --once /ros_robot_controller/bus_servo/set_position \
ros_robot_controller_msgs/msg/ServosPosition \
'{"duration": 4,"position": [{"id": 1, "position": 500},{"id": 2, "position": 508},{"id": 3, "position": 120},{"id": 4, "position": 130}]}'

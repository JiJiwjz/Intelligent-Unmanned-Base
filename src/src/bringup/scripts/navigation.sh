#!/bin/bash
# 启动多点导航
source $HOME/ros_ws/.zshrc # 加载环境
sudo systemctl stop start_app_node
killall -9 rosmaster
roslaunch navigation navigation.launch map:=explore robot_name:=/ master_name:=/ &
sleep 10
roslaunch navigation publish_point.launch enable_navigation:=false robot_name:=/ master_name:=/ &
rviz rviz -d  $HOME/ros_ws/src/navigation/rviz/navigation_desktop.rviz

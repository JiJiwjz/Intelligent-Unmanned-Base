#!/bin/bash
# 启动explore自主建图
source $HOME/ros_ws/.zshrc # 加载环境
sudo systemctl stop start_app_node
killall -9 rosmaster
roslaunch slam slam.launch slam_methods:=explore robot_name:=/ master_name:=/ &
sleep 10
rviz rviz -d  $HOME/ros_ws/src/slam/rviz/explore_desktop.rviz

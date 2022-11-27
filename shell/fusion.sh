#!/bin/sh
xterm  -e  "source ../devel/setup.bash --extend; roslaunch ouster_ros sensor.launch sensor_hostname:=169.254.140.92 viz:=false timestamp_mode:=TIME_FROM_ROS_TIME" &   
sleep 6
xterm  -e  "source ../devel/setup.bash --extend; roslaunch rgb_camera rgb_image.launch"&  
sleep 3
xterm  -e  "source ../devel/setup.bash --extend; roslaunch rgb_camera fusion.launch"&  
sleep 3
xterm  -e  "source ../devel/setup.bash --extend; rosrun rviz rviz -d ../rviz/fusion.rviz"



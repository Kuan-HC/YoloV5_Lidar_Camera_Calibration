#!/bin/sh
xterm  -e  "source ../devel/setup.bash; roslaunch ouster_ros sensor.launch sensor_hostname:=169.254.140.92 viz:=false timestamp_mode:=TIME_FROM_ROS_TIME" &   
sleep 6
xterm  -e  "source ../devel/setup.bash; roslaunch rgb_camera rgb_image.launch"&  
sleep 3
xterm  -e  "source ../devel/setup.bash; rosrun rviz rviz -d ../rviz/lidar_camera.rviz"



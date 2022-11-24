# Lidar_Projection

<b>OS</b>: Ubuntu 18.04 LTS  

## Goals
Project lidar points to rgb images

## Dependencies

* <b> [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)</b>
* <b>Miniconda  </b>  (optional)  
In this project,  environment <b>`Thermo'</b> is created

* <b>xterm  </b>
> sudo apt-get install xterm

* <b>Libraries  </b>  
This project requires following packages: Ouster Lidar, HIKRobot Camera.  
Make sure all packages are correctly configured. Detailed hardwardsetting are list [below](#1).



## Installation
1. clone this project to home folder and init submodule
> cd ~/Thermal_Cognitive  
git submodule init  
git submodule update

2. Install YoloV5
> conda activate thermo  
cd ~/Thermal_Cognitive/src/yolov5/  
git checkout v6.2  
pip install -r requirements.txt  

3. Install Ouster Lidar
source the ROS environemt
> source /opt/ros/melodic/setup.bash   

catkin_make command from within the catkin workspace

4. Build project

> cd ~/Lidar_Projection  
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release  


5. Launch
> conda activate thermo  
cd ~/Lidar_Projection/shell 
./lidar_camera.sh 


<h2 id="1"> Settings </h1> 

* [HIKRobot RGB Camera](./hardware_setting/hikrobot_rgb.md)
* [YOLOV5](https://github.com/ultralytics/yolov5)
* [Ouster Lidar](./hardware_setting/ouster_lidar_config.md)

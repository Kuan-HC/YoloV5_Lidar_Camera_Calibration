# Lidar_Projection

<b>OS</b>: Ubuntu 18.04 LTS  

## Goals
Utlize rgb, thermo camera and lidar for detection

## Dependencies

* <b> [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)</b>
* <b>Miniconda  </b>  (optional)  
In this project,  environment <b>`Thermo'</b> is created

* <b>xterm  </b>
> sudo apt-get install xterm

* <b>Libraries  </b>  
This project requires following packages: Robosense Lidar, HIKRobot Camera, IFCNN and OpenPCdet.  
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

3.


4. Build project
> cd ~/Thermal_Cognitive  
catkin_make

5. Launch
> conda activate thermo  
cd ~/Thermal_Cognitive/shell 
./dataCapture.sh 

<img src="hardware_setting/img/readme_2.png" width = "800">

<h2 id="1"> Settings </h1> 

* [HIKRobot RGB Camera](./hardware_setting/hikrobot_rgb.md)
* [YOLOV5](https://github.com/ultralytics/yolov5)
* [Ouster Lidar Device](./hardware_setting/ouster_lidar_config.md)

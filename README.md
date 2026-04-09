# ROS2 Packages for Tracer Mobile Robot

## Packages

This repository contains packages to control the tracer.

* my_cartographer_launch: a ROS2 package to run cartographer (contains custom lua file for Tracer)
* my_nav2_pkg: a ROS2 package to run the navigation stack (contains custom yaml file for Tracer)
* tracer_description: a ROS2 package containing a simple model of the Tracer (used in the nav stack)
* [Tracer_ros2](https://github.com/ckwan02/tracer_ros2)
    * tracer_base: a ROS wrapper around [ugv_sdk](https://github.com/agilexrobotics/ugv_sdk) to monitor and control the tracer robot
    * tracer_msgs: tracer related message definitions
* [ugv_sdk](https://github.com/agilexrobotics/ugv_sdk)
* [Simple-2D-LiDAR-Odometry](https://github.com/dawan0111/Simple-2D-LiDAR-Odometry): turns LiDAR readings into odometry values
* [sllidar_ros2](https://github.com/Slamtec/sllidar_ros2): starts the LiDAR and starts publishing data

## Supported Hardware

* Tracer
* Tracer-mini



## Installation
 
### 1. Clone the repository (including submodules)
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/mkeehn211/AgileX_Tracer.git
```
 
> **Important:** You must use `--recurse-submodules` to pull the external packages. If you already cloned without it, run:
> ```bash
> git submodule update --init --recursive
> ```
 
### 2. Build the workspace
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Basic usage of the ROS packages

### 1. Setup CAN-To-USB adapter

* Enable gs_usb kernel module(If you have already added this module, you do not need to add it)
    ```
    sudo modprobe gs_usb
    ```
    
* first time use tracer-ros2 package
   ```
   cd ~/ros2_ws/src/ugv_sdk/scripts/
   bash setup_can2usb.bash
   ```
   
* if not the first time use tracer-ros2 package(Run this command every time you turn off the power) 
   ```
   cd ~/ros2_ws/src/ugv_sdk/scripts/
   bash bringup_can2usb_500k.bash
   ```
   
* Testing command
    ```
    # receiving data from can0
    candump can0
    ```
### 2. Launch ROS nodes
 
* Start the base node for the Tracer robot

    ```
    ros2 run tracer_base tracer_base_node
    ```

* Then you can send command to the robot
    ```
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
    x: 0.0
    y: 0.0
    z: 0.0
    angular:
    x: 0.0
    y: 0.0
    z: 0.0" 
    ```
* You can use the teleop test to control the robot with your keyboard
    ```
    ros2 run tracer_base teleop_test
    
    ```
### 3. Running the sensors

* To run the LiDAR (the nav2 package does this already)
    ```
    ros2 run sllidar_ros2 sllidar_node
    
    ```
* To run the IMU
   1. Install pyserial
  
   ```
    sudo apt install python3-serial
    
    ```
   2. Bind port
   ```
   cd ros2_ws/src wit_ros2_imu
   sudo chmod +x bind_usb.sh
   sudo ./bind_usb.sh
    
    ```
   3. Build the worksapce and launch the IMU

    ```
    colcon build
    source install/setup.bash
    ros2 launch wit_ros2_imu rviz_and_imu.launch.py
    
    ```
    
### 4. Navigation stack

* Launch the nav stack in a seperate terminal (the tracer base node should also be running)
    ```
    ros2 launch my_nav2_pkg bringup_launch.py
    
    ```
* Launch rviz in a seperate terminal
    ```
    ros2 launch nav2_bringup rviz_launch.py
    
    ```
* Once Rviz pulls up set an initial position for the robot. Now the robot can navigate on its own.
    
**SAFETY PRECAUSION**: 

Always have your remote controller ready to take over the control whenever necessary. 
You can flip the SWB switch to remote control mode to cut off any velocity commands from CAN communication

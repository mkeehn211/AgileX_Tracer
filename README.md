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

## Basic usage of the ROS packages

1. Clone the packages into your colcon workspace and compile

    (the following instructions assume your catkin workspace is at: ~/ros2_ws/src)

    ```
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://github.com/mkeehn211/AgileX_Tracer.git
    cd ..
    colcon build
    ```
2. Setup CAN-To-USB adapter

* Enable gs_usb kernel module(If you have already added this module, you do not need to add it)
    ```
    sudo modprobe gs_usb
    ```
    
* first time use tracer-ros2 package
   ```
   cd ~/your_ws/src/ugv_sdk/scripts/
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
3. Launch ROS nodes
 
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
3. Using the cartographer

* Connect the lidar to your computer (the tracer base node should also be running)
    ```
    ros2 run sllidar_ros2 sllidar_node
    
    ```

* To run cartographer
    ```
    ros2 launch my_cartographer_launch cartographer_launch.py
    
    ```
* Saving the map
    ```
    ros2 run nav2_map_server map_saver_cli -f ~/my_map
    
    ```
4. Navigation stack

* Connect the lidar to your computer (the tracer base node should also be running)
    ```
    ros2 run sllidar_ros2 sllidar_node
    
    ```
* Run the lidar odom
    ```
    ros2 run lidar_odometry lidar_odometry_node
    
    ```
* Launch the nav stack
    ```
    ros2 launch my_nav2_pkg bringup_launch.py
    
    ```
* Launch rviz to begin navigation
    ```
    ros2 launch nav2_bringup rviz_launch.py
    
    ```
* In rviz you can set an intial orientation (2D Pose Estimate) and set a goal point (Nav2 Goal)
    
**SAFETY PRECAUSION**: 

Always have your remote controller ready to take over the control whenever necessary. 
You can flip the SWB switch to remote control mode to cut off any velocity commands from CAN communication

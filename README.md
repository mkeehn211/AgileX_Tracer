1. Start the tracerx as usual:
sudo modprobe gs_usb

cd ~/ros2_ws/src/ugv_sdk/scripts/
bash bringup_can2usb_500k.bash

ros2 run tracer_base tracer_base_node

3. Run cartographer
ros2 launch my_cartographer_launch cartographer_launch.py

4. Save the map:
ros2 run nav2_map_server map_saver_cli -f ~/my_map

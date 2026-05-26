# Carto_ROS_Mapping

For launching

pkill -f slam_toolbox
pkill -f static_transform_publisher
pkill -f nav2
pkill -f amcl
pkill -f map_server
pkill -f lifecycle_manager
pkill -f rf2o
pkill -f simple_lidar

Terminal 1 - LiDAR decoder

source /opt/ros/humble/setup.bash
source ~/lidar_ws/install/setup.bash

ros2 run simple_lidar reader --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baudrate:=115200 \
  -p frame_id:=laser
  
  
Terminal 2 - base_link -> laser

source /opt/ros/humble/setup.bash

ros2 run tf2_ros static_transform_publisher \
  0 0 0.18 0 0 0 base_link laser
  
  
  
Terminal 3 - RF20

source /opt/ros/humble/setup.bash
source ~/lidar_ws/install/setup.bash

ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py \
  laser_scan_topic:=/scan
  
  
Terminal 4 - SLAM toolbox

source /opt/ros/humble/setup.bash
source ~/lidar_ws/install/setup.bash

ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/home/khouly/lidar_ws/config/slam_clean.yaml
  
  
Terminal 5 - RViz

source /opt/ros/humble/setup.bash
rviz2


To save the map run 

ros2 run nav2_map_server map_saver_cli -f then map location and name


  

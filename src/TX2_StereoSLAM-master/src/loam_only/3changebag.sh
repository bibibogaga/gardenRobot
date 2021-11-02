#!/bin/bash

cd /home/yoga/my_work/livox_formid-40
source devel/setup.bash
# 将lvx格式的数据转成bag
roslaunch livox_ros_driver lvx_to_rosbag.launch lvx_file_path:="/home/yoga/my_work/livox_formid-40/src/Livox-SDK-master/build/sample/lidar_lvx_file/test.lvx" 
rosbag reindex test.bag.active
rosbag fix test.bag.active test.bag



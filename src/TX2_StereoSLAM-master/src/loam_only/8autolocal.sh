#!/bin/bash
cd /home/yoga/my_work/catkin_yoga/src/TX2_StereoSLAM-master/src/loam_only/loam_only_relocal/src/livox_relocalization-master
sudo chmod 777 map
cd map
sudo chmod 777 *
cd /home/yoga/my_work/catkin_yoga/src/TX2_StereoSLAM-master/src/loam_only/loam_only_relocal
source devel/setup.bash
roslaunch livox_relocalization livox_relocalization.launch

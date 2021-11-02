#!/bin/bash

map_dir="/home/yoga/my_work/catkin_yoga/src/TX2_StereoSLAM-master/src/loam_only/loam_only_mapping/src/livox_mapping-master/map/"

#检查文件是否存在，若存在就删除再重建一个
if [ -d "$map_dir" ];then
  echo "文件夹存在, 正在新建文件夹"
  rm -r $map_dir
  mkdir -p $map_dir
fi
cd loam_only_mapping
source devel/setup.bash
roslaunch livox_mapping mapping_mid.launch



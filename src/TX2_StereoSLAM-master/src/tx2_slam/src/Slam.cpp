/////////////////////////////////////////////////////////////////////////

// All copyRights reserved
// Author: Qi
// Date: 2020:05:07
// contract me by: qi.shield95@foxmail.com
// This module use orbSlam2 and zed2 to build a map and save map

///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020, Qi.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

#include "../include/MapBuild.h"
#include <cstdlib>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;
using namespace tx2slam;

void MapBuild::carTF_orb_Callback(const geometry_msgs::PoseStamped::ConstPtr& pose){
  carTF_orb = *pose;
}

void MapBuild::trackingState_Callback(const std_msgs::Int32::ConstPtr& trackingStateMsg)
{
  trackingState = *trackingStateMsg;
}

void MapBuild::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  imu_Msg = *msg;
}

void MapBuild::QTUI_cmd_Callback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
  savemapFlag = msg->data.at(0);
  mappingStatusCmd = msg->data.at(1);

//  goalPoseStamped.pose.position.x = msg->data.at(4)/1000.0;
//  goalPoseStamped.pose.position.y = msg->data.at(5)/1000.0;

//  if(msg->data.at(2) != 0){
//    goalSet = msg->data.at(2);
//  }

//  cmd_vel = msg->data.at(3)/100.0;

  if(savemapFlag == 1){

    if (client.call(srv))
      ROS_INFO("Ready to create Map");
    else
      ROS_ERROR("Failed to call service SaveMap");

    PointCloudToPCD saveAsPcd;
    pcl::PCLPointCloud2ConstPtr mPointcloudFusedMsg_Ptr(mPointcloudFusedMsg_pointer);
    if( !mPointcloudFusedMsg ) {
        mPointcloudFusedMsg = boost::make_shared<sensor_msgs::PointCloud2>();
    }
    pcl_conversions::toPCL(*mPointcloudFusedMsg, *mPointcloudFusedMsg_pointer);
    saveAsPcd.save(mPointcloudFusedMsg_Ptr);

  }
}

void MapBuild::buildMap_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{

    ROS_INFO("go into filter:%d",count);
    count++;
        pointCloud_pub.publish(cloud);
        // Perform the actual filtering
        // VoxelGrid(decrease the memory occupation) & PassThrough(delete some incorrect points)
        pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2;
        pcl::PCLPointCloud2ConstPtr cloudPtr(cloud2);
        pcl_conversions::toPCL(*cloud, *cloud2);
        // VoxelGrid 体素滤波
        pcl::PCLPointCloud2* cloud_filtered_1 = new pcl::PCLPointCloud2;
        pcl::PCLPointCloud2ConstPtr cloud_filter_1_Ptr(cloud_filtered_1);
        pcl::VoxelGrid<pcl::PCLPointCloud2> filter_1;
        filter_1.setInputCloud (cloudPtr);
        filter_1.setLeafSize (VoxelGrid_LeafSize, VoxelGrid_LeafSize, VoxelGrid_LeafSize);
        filter_1.filter(*cloud_filtered_1);
        // PassThrough 直通滤波（）
//        pcl::PCLPointCloud2* cloud_filtered_2 = new pcl::PCLPointCloud2;
//        pcl::PCLPointCloud2ConstPtr cloud_filter_2_Ptr(cloud_filtered_2);
        pcl::PassThrough<pcl::PCLPointCloud2> filter_2;
//        filter_2.setInputCloud (cloud_filter_1_Ptr);
//        filter_2.setFilterFieldName ("y");
//        filter_2.setFilterLimits (PassThrough_yMin, PassThrough_yMax);
//    //    filter_2.setFilterLimitsNegative (true);
//        filter_2.filter(*cloud_filtered_2);

        pcl::PCLPointCloud2* cloud_filtered_3 = new pcl::PCLPointCloud2;
        pcl::PCLPointCloud2ConstPtr cloud_filter_3_Ptr(cloud_filtered_3);
        filter_2.setInputCloud (cloud_filter_1_Ptr);
        filter_2.setFilterFieldName ("z");
        filter_2.setFilterLimits (PassThrough_zMin, PassThrough_zMax);
    //    filter_2.setFilterLimitsNegative (true);
        filter_2.filter(*cloud_filtered_3);


        pcl::PCLPointCloud2 cloud_filtered_4;
        // StatisticalOutlierRemoval
        pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> filter_3;
        filter_3.setInputCloud (cloud_filter_3_Ptr);
        filter_3.setMeanK (StatisticalOutlierRemoval_MeanK);
        filter_3.setStddevMulThresh (StatisticalOutlierRemoval_StddevMulThresh);
        filter_3.filter (cloud_filtered_4);

        pointCloudFused_pub.publish(cloud_filtered_4);
//    pcl::fromPCLPointCloud2 (cloud_filtered_4, *cloud_xyz);
//    std::cerr << "Cloud after filtering: " << std::endl;
//    for (size_t i = 0; i < cloud_xyz->width; ++i)
//        std::cerr << "    " << cloud_xyz->at(i).z << std::endl;
        if ((cloud_filtered_4.width * cloud_filtered_4.height) == 0)
            return;
        pcl::fromPCLPointCloud2 (cloud_filtered_4, *cloud_xyz);
        // ... read or fill in source and target
        if(cloud_xyzFused->width == 0){
            pcl::PointXYZRGB pointFill = cloud_xyz->at(0);
            cloud_xyzFused->push_back(pointFill);
        }
        pcl::registration::CorrespondenceEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB> est;
        cloud_xyzFusedPtr = cloud_xyzFused->makeShared();
        cloud_xyzPtr = cloud_xyz->makeShared();
        est.setInputSource (cloud_xyzPtr);
        est.setInputTarget (cloud_xyzFusedPtr);
        pcl::Correspondences all_correspondences;
        // Determine all reciprocal correspondences
        est.determineReciprocalCorrespondences (all_correspondences);

        // filter the reciprocal points cloud  点云融合
        if(1)
        {
            // fused the current cloud to the fused cloud
            *cloud_xyzFused += *cloud_xyz;
            if( !mPointcloudFusedMsg ) {
                mPointcloudFusedMsg = boost::make_shared<sensor_msgs::PointCloud2>();
            }
            pcl::toROSMsg(*cloud_xyzFused, *mPointcloudFusedMsg);
            mPointcloudFusedMsg->header.frame_id = "map";
            //pointCloudFused_pub.publish(mPointcloudFusedMsg);
        }


}

void MapBuild::init()
{
  ros::init (init_argc, init_argv, "Slam");
  ros::NodeHandle n;

  ROS_INFO("/*************\nSlam Parameters:\n--------------------------------------");
  n.getParam("Slam/VoxelGrid_LeafSize", VoxelGrid_LeafSize);
  ROS_INFO("VoxelGrid_LeafSize: %f",VoxelGrid_LeafSize);
  n.getParam("Slam/PassThrough_yMin", PassThrough_yMin);
  ROS_INFO("PassThrough_yMin: %f",PassThrough_yMin);
  n.getParam("Slam/PassThrough_yMax", PassThrough_yMax);
  ROS_INFO("PassThrough_yMax: %f",PassThrough_yMax);
  n.getParam("Slam/PassThrough_zMin", PassThrough_zMin);
  ROS_INFO("PassThrough_zMin: %f",PassThrough_zMin);
  n.getParam("Slam/PassThrough_zMax", PassThrough_zMax);
  ROS_INFO("PassThrough_zMax: %f",PassThrough_zMax);
  n.getParam("Slam/StatisticalOutlierRemoval_MeanK", StatisticalOutlierRemoval_MeanK);
  ROS_INFO("StatisticalOutlierRemoval_MeanK: %f",StatisticalOutlierRemoval_MeanK);
  n.getParam("Slam/StatisticalOutlierRemoval_StddevMulThresh", StatisticalOutlierRemoval_StddevMulThresh);
  ROS_INFO("StatisticalOutlierRemoval_StddevMulThresh: %f",StatisticalOutlierRemoval_StddevMulThresh);
  n.getParam("Slam/TimeLimit", TimeLimit);
  ROS_INFO("TimeLimit: %f",TimeLimit);
  n.getParam("Slam/PoseBiasLimit", PoseBiasLimit);
  ROS_INFO("PoseBiasLimit: %f",PoseBiasLimit);
  n.getParam("Slam/angular_velocityLimit", angular_velocityLimit);
  ROS_INFO("angular_velocityLimit: %f",angular_velocityLimit);

  // publish Fused cloud
  pointCloudFused_pub = n.advertise<sensor_msgs::PointCloud2>("/mapBuild/cloud_Fused", 1);
  pointCloud_pub = n.advertise<sensor_msgs::PointCloud2>("/repubcloud", 1);
  // subscribe pose and imu msg
  imu_sub = n.subscribe("/imu/data", 1, &MapBuild::imuCallback,this);
  carTF_orb_sub = n.subscribe("/loam_pose", 1, &MapBuild::carTF_orb_Callback,this);
  //trackingStat_sub = n.subscribe("/orb_slam2_stereo/trackingState", 1, &MapBuild::trackingState_Callback,this);

  // set a Timer to call path-plan function
//  navigationCtrlTimer = n.createTimer(ros::Duration(2.5), &MapBuild::navigation_Callback,this);

  // receive the UI cmd
  QTUI_cmd_sub = n.subscribe("/rover/QtUI_cmd_Msg", 1, &MapBuild::QTUI_cmd_Callback,this);

  pointCloud_sub = n.subscribe("/laser_cloud_surround", 1, &MapBuild::buildMap_callback,this);
  // receive the origin cloud and camera pose 可以替代当前点云和姿态
 /* pointCloud_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>
      ( n, "/laser_cloud_surround", 1);
  carTF_zed2_sub = new message_filters::Subscriber<geometry_msgs::PoseStamped>
      (n, "/loam_pose", 1);
  // 实时的点云和姿态信息需要时间戳同步，不然融合出来的地图会发生很大的偏移和扭曲
  sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *pointCloud_sub, *carTF_zed2_sub);
  sync_->registerCallback(boost::bind(&MapBuild::buildMap_callback, this, _1, _2));
*/
  // call save map service
  /*client = n.serviceClient<orb_slam2_ros::SaveMap>("/orb_slam2_stereo/save_map");
  srv.request.name = "/home/yoga/my_work/catkin_qi/src/TX2_StereoSLAM-master/src/tx2_slam/map/bin/zed2_slam_Map.bin";
*/
  ros::spin();
}

int main(int argc, char** argv)
{
  MapBuild map(argc,argv);
}

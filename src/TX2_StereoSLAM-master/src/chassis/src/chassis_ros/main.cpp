#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_broadcaster.h>

#include "chassis_control.h"

#define CMD_TIME_OUT 300  // ms

static float aim_x_vel = 0.0;
static float aim_z_omega = 0.0;
static ros::Time cmd_update_time;

void cmdMsgCallback(const geometry_msgs::Twist &cmd)
{
  aim_x_vel = cmd.linear.x;
  aim_z_omega = cmd.angular.z;
  cmd_update_time = ros::Time::now();
}

void alarmCallback(const std_msgs::Bool &alarm) { setAlarm(alarm.data); }

bool charge(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  res.success = setCharge((int)(req.data));
  return true;
}

float shortestAngleDist(float angle)
{
  if (angle > M_PI)
  {
    angle = angle - 2.0 * M_PI;
  }
  else if (angle < -M_PI)
  {
    angle = angle + 2.0 * M_PI;
  }
  return angle;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "chassis_node");
  ros::NodeHandle n;
  ros::NodeHandle private_n("~");
  bool publish_tf;
  std::string odom_topic;

  private_n.param("publish_tf", publish_tf, false);
  private_n.param("odom_topic", odom_topic, std::string("/odom_chassis"));

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>(odom_topic, 1);
  ros::Publisher battery_pub = n.advertise<sensor_msgs::BatteryState>("/battery", 1);
  ros::Subscriber cmd_sub = n.subscribe("/cmd_vel", 1, cmdMsgCallback);
  ros::Subscriber alarm_sub = private_n.subscribe("alarm", 1, alarmCallback);
  ros::ServiceServer server = private_n.advertiseService("charge", charge);

  tf::TransformBroadcaster odom_broadcaster;
  ros::Time current_time;
  ros::Time last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  if (!chassisInit())
  {
    ROS_ERROR("chassis init error");
    return 0;
  }

  int cnt = 0;
  float x = 0.0f;
  float y = 0.0f;
  float theta = 0.0f;
  float x_vel = 0.0f;
  float z_omega = 0.0f;

  ros::Rate r(20.0);
  bool first_odom = true;
  float pose[3] = {0};
  while (n.ok())
  {
    ros::spinOnce();  // check for incoming messages
    current_time = ros::Time::now();

    cnt++;
    if (cnt % 10 == 0)
    {
      sensor_msgs::BatteryState state;
      state.header.stamp = current_time;
      int percentage = 0;
      if(getBatteryInfo(state.voltage, state.current, percentage))
      {
        state.percentage = percentage;
        battery_pub.publish(state);
      }
    }

    if (current_time - cmd_update_time > ros::Duration(CMD_TIME_OUT / 1E3))
    {
      aim_x_vel = 0.0f;
      aim_z_omega = 0.0f;
    }

    getChassisInfo(x_vel, z_omega);
    getChassisOdom(x, y, theta);
    chassisControl(aim_x_vel, aim_z_omega);
    if (first_odom)
    {
      pose[0] = x;
      pose[1] = y;
      pose[2] = theta;
      first_odom = false;
    }
    // since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat =
        tf::createQuaternionMsgFromYaw(shortestAngleDist(theta - pose[2]));
    // first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x =
        cos(-pose[2]) * (x - pose[0]) - sin(-pose[2]) * (y - pose[1]);
    odom_trans.transform.translation.y =
        sin(-pose[2]) * (x - pose[0]) + cos(-pose[2]) * (y - pose[1]);
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    // send the transform
    if (publish_tf)
    {
      odom_broadcaster.sendTransform(odom_trans);
    }
    // next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    // set the position
    odom.pose.pose.position.x = odom_trans.transform.translation.x;
    odom.pose.pose.position.y = odom_trans.transform.translation.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = x_vel;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = z_omega;

    odom_pub.publish(odom);
    last_time = current_time;
    r.sleep();
  }
}

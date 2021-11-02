#include "chassis_control.h"

#include <ros/ros.h>

#include <memory>

std::shared_ptr<ChassisSerial> chassis_driver;

bool chassisInit()
{
  chassis_driver = std::make_shared<ChassisSerial>();
  if (chassis_driver == nullptr)
  {
    ROS_ERROR("malloc chassis serial error.");
    return false;
  }

  if (!chassis_driver->init())
  {
    ROS_ERROR("chassis init error.");
    return false;
  }
  return true;
}

bool getBatteryInfo(float &vol, float &cur, int &percent)
{
  if (!chassis_driver->getEntry(ChassisSerial::OdIndex::INDEX_SYS_VOLTAGE, vol))
  {
    ROS_WARN("get index failed %d",
             int(ChassisSerial::OdIndex::INDEX_SYS_VOLTAGE));
    return false;
  }
  if (!chassis_driver->getEntry(ChassisSerial::OdIndex::INDEX_SYS_CURRENT, cur))
  {
    ROS_WARN("get index failed %d",
             int(ChassisSerial::OdIndex::INDEX_SYS_CURRENT));
    return false;
  }
  if (!chassis_driver->getEntry(ChassisSerial::OdIndex::INDEX_SYS_POWER_PERCENTAGE,
                                percent))
  {
    ROS_WARN("get index failed %d",
             int(ChassisSerial::OdIndex::INDEX_SYS_POWER_PERCENTAGE));
    return false;
  }
  return true;
}

bool getChassisCollision(int &collision)
{
  if (!chassis_driver->getEntry(ChassisSerial::OdIndex::INDEX_SYS_COLLISION,
                                collision))
  {
    ROS_WARN("get index failed %d",
             int(ChassisSerial::OdIndex::INDEX_SYS_COLLISION));
    return false;
  }
  return true;
}

bool getChassisState(SystemState &state)
{
  if (!chassis_driver->getEntry(ChassisSerial::OdIndex::INDEX_SYS_STATE,
                                (int &)state))
  {
    ROS_WARN("get index failed %d",
             int(ChassisSerial::OdIndex::INDEX_SYS_STATE));
    return false;
  }
  return true;
}

bool setText(const char *text)
{
  if (!chassis_driver->setEntry(ChassisSerial::OdIndex::INDEX_SYS_TEXT, text))
  {
    ROS_WARN("set index failed %d",
             int(ChassisSerial::OdIndex::INDEX_SYS_TEXT));
    return false;
  }
  return true;
}

bool setCharge(int charge)
{
  if (!chassis_driver->setEntry(ChassisSerial::OdIndex::INDEX_SYS_CHARGE,
                                charge))
  {
    ROS_WARN("set index failed %d",
             int(ChassisSerial::OdIndex::INDEX_SYS_CHARGE));
    return false;
  }
  return true;
}

bool setAlarm(int alarm)
{
  if (!chassis_driver->setEntry(ChassisSerial::OdIndex::INDEX_SYS_ALARM, alarm))
  {
    ROS_WARN("set index failed %d",
             int(ChassisSerial::OdIndex::INDEX_SYS_ALARM));
    return false;
  }
  return true;
}

bool setLight(int light)
{
  if (!chassis_driver->setEntry(ChassisSerial::OdIndex::INDEX_SYS_LIGHT, light))
  {
    ROS_WARN("set index failed %d",
             int(ChassisSerial::OdIndex::INDEX_SYS_LIGHT));
    return false;
  }
  return true;
}


/**
 * @brief 控制车的x方向的线速度和z方向的转动速度
 * @param aim_x_vel  m/s
 * @param aim_z_omega rad/s
 */
bool chassisControl(float aim_x_vel, float aim_z_omega)
{
  if (!chassis_driver->setEntry(ChassisSerial::OdIndex::INDEX_AIM_CHASSIS_VEL,
                                aim_x_vel))
  {
    ROS_WARN("set index failed %d",
             int(ChassisSerial::OdIndex::INDEX_AIM_CHASSIS_VEL));
    return false;
  }
  if (!chassis_driver->setEntry(
          ChassisSerial::OdIndex::INDEX_AIM_CHASSIS_POS_OR_OMEGA, aim_z_omega))
  {
    ROS_WARN("set index failed %d",
             int(ChassisSerial::OdIndex::INDEX_AIM_CHASSIS_POS_OR_OMEGA));
    return false;
  }
  return true;
}
/**
 * @brief 更新x线速度和z转速度
 * @param x_vel m/s
 * @param z_omega rad/s
 */
bool getChassisInfo(float &x_vel, float &z_omega)
{
  if (!chassis_driver->getEntry(ChassisSerial::OdIndex::INDEX_CHASSIS_VEL,
                                x_vel))
  {
    ROS_WARN("get index failed %d",
             int(ChassisSerial::OdIndex::INDEX_CHASSIS_VEL));
    return false;
  }
  if (!chassis_driver->getEntry(
          ChassisSerial::OdIndex::INDEX_CHASSIS_POS_OR_OMEGA, z_omega))
  {
    ROS_WARN("get index failed %d",
             int(ChassisSerial::OdIndex::INDEX_CHASSIS_POS_OR_OMEGA));
    return false;
  }
  return true;
}
/**
 * @brief 更新里程计的信息
 * @param odom_x
 * @param odom_y
 * @param odom_theta 转角
 */
bool getChassisOdom(float &odom_x, float &odom_y, float &odom_theta)
{
  if (!chassis_driver->getEntry(ChassisSerial::OdIndex::INDEX_CHASSIS_ODOM_X,
                                odom_x))
  {
    ROS_WARN("get index failed %d",
             int(ChassisSerial::OdIndex::INDEX_CHASSIS_ODOM_X));
    return false;
  }
  if (!chassis_driver->getEntry(ChassisSerial::OdIndex::INDEX_CHASSIS_ODOM_Y,
                                odom_y))
  {
    ROS_WARN("get index failed %d",
             int(ChassisSerial::OdIndex::INDEX_CHASSIS_ODOM_Y));
    return false;
  }
  if (!chassis_driver->getEntry(
          ChassisSerial::OdIndex::INDEX_CHASSIS_ODOM_THETA, odom_theta))
  {
    ROS_WARN("get index failed %d",
             int(ChassisSerial::OdIndex::INDEX_CHASSIS_ODOM_THETA));
    return false;
  }
  return true;
}

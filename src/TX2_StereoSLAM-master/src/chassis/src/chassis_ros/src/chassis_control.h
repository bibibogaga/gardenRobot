#ifndef __CHASSIS_CONTROL_H_
#define __CHASSIS_CONTROL_H_

#include <string>

#include "chassis_serial.h"

bool chassisInit();
bool chassisControl(float aim_x_vel, float aim_z_omega);
bool getChassisInfo(float &x_vel, float &z_omega);
bool getChassisOdom(float &odom_x, float &odom_y, float &odom_theta);

bool getBatteryInfo(float &vol, float &cur, int &percent);
bool getChassisState(SystemState &state);
bool getChassisCollision(int &collision);

// 11个汉字(GB2312编码)或者23个字符,
bool setText(const char *text);

// 0 / 1
bool setCharge(int charge);

// 0 / 1
bool setAlarm(int alarm);

// 0 ~ 100
bool setLight(int light);

#endif
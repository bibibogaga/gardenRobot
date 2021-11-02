#ifndef __CHASSIS_SERIAL_H_
#define __CHASSIS_SERIAL_H_

#include <memory>

class SerialClient;

typedef enum
{
  SYS_INIT = 0x00,
  SYS_NORMAL,
  SYS_REMOTE,
  SYS_ESTOP,
  SYS_CALIB,
  SYS_TEST,
  SYS_CHARGING,

  SYS_ERR = 0x10,
  SYS_ERR_ID,
  SYS_ERR_CAN,
  SYS_ERR_485,
  SYS_ERR_CHARGING,
  SYS_ERR_LOW_VOLTAGE,
  SYS_ERR_OVER_VOLTAGE,
  SYS_ERR_OVER_CURRENT,
  SYS_ERR_OVER_TEMP,

  SYS_STATE_LEN,
} SystemState;

class ChassisSerial
{
 public:
  enum OdIndex
  {
    INDEX_SYS_STATE = 0,
    INDEX_SYS_POWER_PERCENTAGE = 1,

    INDEX_CHASSIS_VEL = 2,
    INDEX_CHASSIS_POS_OR_OMEGA = 3,
    INDEX_CHASSIS_ODOM_X = 4,
    INDEX_CHASSIS_ODOM_Y = 5,
    INDEX_CHASSIS_ODOM_THETA = 6,

    INDEX_SYS_VOLTAGE = 7,
    INDEX_SYS_CURRENT = 8,

    INDEX_AIM_CHASSIS_VEL = 9,
    INDEX_AIM_CHASSIS_POS_OR_OMEGA = 10,

    INDEX_SYS_CHARGE = 11,
    INDEX_SYS_ALARM = 12,
    INDEX_SYS_TEXT = 13,
    INDEX_SYS_LIGHT = 14,
    INDEX_SYS_COLLISION = 15,
  };

  ChassisSerial();
  ~ChassisSerial();

  bool init();
  bool getEntry(OdIndex index, int &data);
  bool getEntry(OdIndex index, float &data);
  bool getEntry(OdIndex index, char *data);
  bool setEntry(OdIndex index, int data);
  bool setEntry(OdIndex index, float data);
  bool setEntry(OdIndex index, const char *data);

 private:
  void handleTimeout();

  std::shared_ptr<SerialClient> serial_client_;
  int conn_timeout_cnt_;
  std::string name_;
  int baud_rate_;
};

#endif  // __CHASSIS_SERIAL_H_

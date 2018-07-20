#ifndef DXL_HANDLER_H_
#define DXL_HANDLER_H_s

#include <ros/ros.h>
#include <string>
#include <sensor_msgs/JointState.h>
#include <map>
#include <dynamixel_sdk/dynamixel_sdk.h> // Uses Dynamixel SDK library
#include <realtime_tools/realtime_publisher.h>


using namespace std;


#define ADDR_PRO_TORQUE_ENABLE          562                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          596
#define ADDR_PRO_GOAL_VELOCITY          600

#define ADDR_X_TORQUE_ENABLE            64 // 1
#define ADDR_X_GOAL_VELOCITY            104 // 4
#define ADDR_X_GOAL_POSITION            116 //4
#define ADDR_X_PRESENT_POSITION            132 // 4

#define HEAD_LIDAR_VELOCITY 90//60
//#define HEAD_LIDAR_VELOCITY 59//60 for calibration
//#define HEAD_LIDAR_VELOCITY 24


class DXLHandler
{
public:
  DXLHandler(const char* port_name, float protocol_version, int baud_rate);

  void open(int baud);
  void setTorqueX(int id, int enable);
  void setTorquePro(int id, int enable);
  void setPositionPro(int id, int position);
  void setPositionX(int id, int position);
  void setVelocityX(int id, int velocity);
  void allTorqueOn();
  void allTorqueOff();
  void setInitCommand();

  void controlLoop();

  void commandCallback(const sensor_msgs::JointState::ConstPtr & msg);

private:
  ros::NodeHandle nh;
  ros::Publisher joint_pub_;
  ros::Subscriber joint_command_sub_;

  sensor_msgs::JointState joint_msg_;

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  double presentPosition_;

  uint8_t error_;
  int comm_result_;

  std::map<std::string, int> joint_name_map_;
  std::map<std::string, int> hand_motor_offset_; ///< (unit: digit (dxl))
  std::map<std::string, double> hand_motor_sign_;
  std::map<std::string, double> hand_motor_min_;  ///< (unit: radian)
  std::map<std::string, double> hand_motor_max_;  ///< (unit: radian)

};



#endif

/**
 * @file /include/dyros_jet_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef dyros_jet_gui_STATUS_QNODE_HPP_
#define dyros_jet_gui_STATUS_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <vector>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <smach_msgs/SmachContainerStatus.h>

#endif


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dyros_jet_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class StatusQNode : public QThread {
  Q_OBJECT
public:
  StatusQNode(int argc, char** argv );
  virtual ~StatusQNode();
  bool init();
  void init_nh();
  void run();

  void send_transition(std::string str);

  void send_hello_count(const int count);
  void shutdown();

  std_msgs::Float32MultiArray recog_info_msg_;


  void changeDxlMode(int mode);
  void setAimPosition(int id, double radian);
  void setTorque(int value);

  std::string getCurrentState() const {return current_state;}

Q_SIGNALS:
  void stateUpdated();
  void recogInfoUpdated();
  void rosShutdown();

private:
  int init_argc;
  char** init_argv;
  ros::Publisher smach_publisher;
  ros::Publisher recog_cmd_publisher;
  ros::Publisher hello_cnt_publisher;
  ros::Publisher shutdown_publisher;

  ros::Subscriber recog_point_subscriber;
  ros::Subscriber smach_subscriber;


  ros::ServiceClient dxl_mode_set_client_; ///< dynamixel mode select service
  ros::ServiceClient dxl_motor_set_client_; ///< dynmamixel motor setting service

  ros::NodeHandle *nh;

  std::string current_state;

  bool isConnected;
  void recogInfoCallback(const std_msgs::Float32MultiArrayConstPtr& msg);
  void stateCallback(const smach_msgs::SmachContainerStatusConstPtr& msg);

};

}  // namespace dyros_jet_gui

#endif /* dyros_jet_gui_STATUS_QNODE_HPP_ */

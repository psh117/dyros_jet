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

#ifndef dyros_jet_gui_JOINT_QNODE_HPP_
#define dyros_jet_gui_JOINT_QNODE_HPP_

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

#include <dyros_jet_msgs/JointState.h>
#include <dyros_jet_msgs/JointCommand.h>
#endif


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dyros_jet_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class JointQNode : public QThread {
    Q_OBJECT
public:
  JointQNode(int argc, char** argv );
  virtual ~JointQNode();
  bool init();
  void init_nh();
  void run();

    void send_joint_ctrl(int id, const char* jointName, double angle);
    void publish_joint_ctrl();

    dyros_jet_msgs::JointState joint_msg_;
    dyros_jet_msgs::JointCommand joint_cmd_msg_;

    std_msgs::String controlbase_data_;

Q_SIGNALS:
    void jointStateUpdated();
    void rosShutdown();

private:
  int init_argc;
    char** init_argv;
    ros::Publisher joint_ctrl_publisher;

    ros::Subscriber joint_state_subscirber;

    ros::NodeHandle *nh;

    bool isConnected;
    void jointStateCallback(const dyros_jet_msgs::JointStateConstPtr& msg);

};

}  // namespace dyros_jet_gui

#endif /* dyros_jet_gui_JOINT_QNODE_HPP_ */

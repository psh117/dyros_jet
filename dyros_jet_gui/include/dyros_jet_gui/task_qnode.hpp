/**
 * @file /include/dyros_jet_gui/task_qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef dyros_jet_gui_TASK_QNODE_HPP_
#define dyros_jet_gui_TASK_QNODE_HPP_

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
#include <std_msgs/Bool.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>

#include <dyros_jet_msgs/JointCommand.h>
#include <dyros_jet_msgs/TaskCommand.h>
#include <dyros_jet_msgs/WalkingCommand.h>
#endif


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dyros_jet_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class TaskQNode : public QThread {
    Q_OBJECT
public:
  TaskQNode(int argc, char** argv );
  virtual ~TaskQNode();
  bool init();
  void init_nh();
  void run();

    void send_ft_calib(float time);
    void send_task_ctrl();
    void publish_joint_ctrl();
    void send_walk_ctrl();
    void send_bool_cb();
    void send_hand_cmd();

    void left_ftStateCallback(const geometry_msgs::WrenchStampedConstPtr& msg);
    void right_ftStateCallback(const geometry_msgs::WrenchStampedConstPtr& msg);

    dyros_jet_msgs::JointCommand joint_cmd_msg_;
    dyros_jet_msgs::TaskCommand task_cmd_msg_;
    dyros_jet_msgs::WalkingCommand walk_cmd_msg_;
    //ft_sensor_msg
    geometry_msgs::WrenchStamped ft_lf_msg_;
    geometry_msgs::WrenchStamped ft_rf_msg_;

    sensor_msgs::JointState hand_cmd_msg_;

    std_msgs::Bool controlbase_bool_;

Q_SIGNALS:
    void rosShutdown();

private:
  int init_argc;
    char** init_argv;
    ros::Publisher joint_ctrl_publisher;

    ros::Publisher walking_cmd_publisher;
    ros::Publisher controlbase_bool_publisher;
    ros::Publisher task_cmd_publisher;

    ros::Publisher hand_cmd_publisher_;

    ros::Publisher ft_sensor_calib_publisher;
    ros::Subscriber ft_sensor_lf_state_subscriber;
    ros::Subscriber ft_sensor_rf_state_subscriber;

    ros::NodeHandle *nh;

    bool isConnected;
};

}  // namespace dyros_jet_gui

#endif /* dyros_jet_gui_TASK_QNODE_HPP_ */

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

#ifndef dyros_jet_gui_QNODE_HPP_
#define dyros_jet_gui_QNODE_HPP_

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

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
  void init_nh();
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

    void send_ft_calib(float time);
    void send_transition(std::string str);
    void send_joint_ctrl(int id, const char* jointName, double angle);
    void send_task_ctrl();
    void publish_joint_ctrl();
    void send_walk_ctrl();
    void send_data_cb();

    // void send_walking_cmd(thormang_ctrl_msgs::WalkingCmd& walking_msg);
    // void send_recog_cmd(thormang_ctrl_msgs::RecogCmd& recog_msg);
    // void send_task_cmd(thormang_ctrl_msgs::TaskCmd& task_msg);
    void send_hello_count(const int count);

    // thormang_ctrl_msgs::JointState joint_msg;
    dyros_jet_msgs::JointState joint_msg_;
    dyros_jet_msgs::JointCommand joint_cmd_msg_;
    dyros_jet_msgs::TaskCommand task_cmd_msg_;
    dyros_jet_msgs::WalkingCommand walk_cmd_msg_;

    std_msgs::Float32MultiArray recog_info_msg_;
    std_msgs::String controlbase_data_;



Q_SIGNALS:

    void jointStateUpdated();
    void recogInfoUpdated();
    void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
    char** init_argv;
    ros::Publisher smach_publisher;
    ros::Publisher joint_ctrl_publisher;

    ros::Publisher walking_cmd_publisher;
    ros::Publisher controlbase_publisher;
    ros::Publisher task_cmd_publisher;
    ros::Publisher recog_cmd_publisher;

    ros::Publisher hello_cnt_publisher;

    ros::Publisher ft_sensor_calib_publisher;

    ros::Subscriber joint_state_subscirber;
    ros::Subscriber recog_point_subscriber;

    ros::NodeHandle *nh;

    QStringListModel logging_model;

    bool isConnected;
    void jointStateCallback(const dyros_jet_msgs::JointStateConstPtr& msg);
    void recogInfoCallback(const std_msgs::Float32MultiArrayConstPtr& msg);

};

}  // namespace dyros_jet_gui

#endif /* dyros_jet_gui_QNODE_HPP_ */

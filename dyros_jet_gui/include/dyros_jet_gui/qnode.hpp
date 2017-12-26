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
    void send_joint_ctrl(int id, double angle);

    // void send_walking_cmd(thormang_ctrl_msgs::WalkingCmd& walking_msg);
    // void send_recog_cmd(thormang_ctrl_msgs::RecogCmd& recog_msg);
    // void send_task_cmd(thormang_ctrl_msgs::TaskCmd& task_msg);
    void send_hello_count(const int count);

    // thormang_ctrl_msgs::JointState joint_msg;
    std_msgs::Float32MultiArray recog_info_msg;


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
    ros::Publisher task_cmd_publisher;
    ros::Publisher recog_cmd_publisher;

    ros::Publisher hello_cnt_publisher;

    ros::Publisher ft_sensor_calib_publisher;

    ros::Subscriber joint_state_subscirber;
    ros::Subscriber recog_point_subscriber;

    ros::NodeHandle *nh;

    QStringListModel logging_model;

    bool isConnected;
    // void jointStateCallback(const thormang_ctrl_msgs::JointStateConstPtr& msg);
    void recogInfoCallback(const std_msgs::Float32MultiArrayConstPtr& msg);

};

}  // namespace dyros_jet_gui

#endif /* dyros_jet_gui_QNODE_HPP_ */

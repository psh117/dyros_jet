#ifndef _CONTROL_BASE_H
#define _CONTROL_BASE_H

// STD Library
#include <vector>

// System Library
#include <termios.h>

// ROS Library
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/transform_datatypes.h>

// ROS Messages
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include "dyros_jet_msgs/JointSet.h"
#include "dyros_jet_msgs/JointState.h"
#include "dyros_jet_msgs/WalkingCmd.h"
#include "dyros_jet_msgs/RecogCmd.h"
#include "dyros_jet_msgs/TaskCmd.h"
#include "dyros_jet_msgs/TaskCmdboth.h"

// User Library
#include "math_type_define.h"
// #include "Walking_Controller.h"
// #include "Upperbody_Controller.h"


namespace dyros_jet_controller
{

using namespace Eigen;
using namespace std;

extern const string JOINT_NAME[40];
extern const int JOINT_ID[40];

enum body_select {UPPER,WAIST, WALKING, HEAD};

class controlBase
{

public:
    controlBase(ros::NodeHandle &nh, double Hz);
    virtual ~controlBase(){}
    // Default User Call function
    void parameter_initialize(); // initialize all parameter function(q,qdot,force else...)
    virtual void readdevice(); // read device means update all subscribed sensor data and user command
    virtual void update(); // update controller based on readdevice
    virtual void compute(); // compute algorithm and update all class object
    virtual void reflect(); // reflect next step actuation such as motor angle else
    virtual void writedevice()=0; // publish to actuate devices
    virtual void wait()=0;  // wait
    double Rounding( double x, int digit );
    int getch();

    bool check_state_changed();

    const double getHz() { return Hz_; }
protected:
    ros::Subscriber smachSub;

    vector<int> jointID;
    vector<int> jointInvID;

    int uiUpdateCount;
    bool isFirstBoot;

    VectorXd q_; // current q
    VectorXd q_dot_; // current qdot
    VectorXd torque_; // current joint toruqe

    Vector6d leftFootFT_; // current left ft sensor values
    Vector6d rightFootFT_; // current right ft sensor values

    Vector3d gyro_; // current gyro sensor values
    Vector3d accelometer_; // current accelometer values

    Matrix3d pelvis_orientation_;

    VectorXd desired_q_; // current desired joint values

    int total_dof;

    int         cnt_;

    VectorXd    target_q_;
    MatrixXd    target_x_;

    void updateDesired(body_select body, VectorXd &update_q);

private:
    double Hz_;

private:

    void make_id_inverse_list();

};

}

#endif

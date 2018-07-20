#ifndef _DYROS_JET_HAPTIC_H
#define _DYROS_JET_HAPTIC_H
    
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <string>

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>

#include <math.h>

#include <dyros_jet_msgs/TaskCommand.h>
#include "dhdc.h"

class DyrosHaptic
    {
   public:
     DyrosHaptic();
     void hapticLoop();
     
   private:
     ros::NodeHandle nh_;
     realtime_tools::RealtimePublisher<dyros_jet_msgs::TaskCommand> haptic_publisher_;
     dyros_jet_msgs::TaskCommand task_cmd_msg_;

bool end_effector_; // 0 for left, 1 for right
bool command_frame_; // 0 for base frame, 1 for end effector frame
bool end_effector_changed_;
double haptic_pos_x_;
double haptic_pos_y_;
double haptic_pos_z_;
double haptic_pos_x_pre_;
double haptic_pos_y_pre_;
double haptic_pos_z_pre_;
double haptic_ang_x_;
double haptic_ang_y_;
double haptic_ang_z_;
bool quit_flag_;
int haptic_button_;
int haptic_button_pre_;
double scale_;
   };
 
int main(int argc, char** argv);

#endif

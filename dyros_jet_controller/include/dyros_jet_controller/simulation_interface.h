
#include "control_base.h"
#include "math_type_define.h"
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
// Used data structures:

namespace dyros_jet_controller
{

extern const string JOINT_NAME[40];
extern const int JOINT_ID[40];

class SimulationInterface : public controlBase{
 public:   
   SimulationInterface(ros::NodeHandle &nh, double Hz); // constructor for initialize node
   virtual ~SimulationInterface() { vrep_stop(); }
   void vrep_start();
   void vrep_stop();
   void vrep_stepDone();
   void vrep_enableSyncMode();

   void update(); // update controller based on readdevice
   void compute(); // compute algorithm and update all class object
   void writedevice(); // publish to actuate devices
   void wait();

private:  // CALLBACK
   void simulationTimeCallback(const std_msgs::Float32ConstPtr& msg);
   void jointCallback(const sensor_msgs::JointStateConstPtr& msg);
   void leftFTCallback(const geometry_msgs::WrenchStampedConstPtr& msg);
   void rightFTCallback(const geometry_msgs::WrenchStampedConstPtr& msg);
   void imuCallback(const sensor_msgs::ImuConstPtr& msg);

 private:

   ros::Publisher vrepJointSetPub_;
   ros::Publisher vrepSimStartPub_;
   ros::Publisher vrepSimStopPub_;
   ros::Publisher vrepSimStepDonePub_;
   ros::Publisher vrepSimEnableSyncModePub_;

   sensor_msgs::JointState jointSetMsg_;


   bool simulationRunning_;
   float simulationTime_; // current simulation time

   ros::Rate rate_;

   ros::Subscriber vrepSimStateSub_;

   ros::Subscriber imuSub_;
   ros::Subscriber jointSub_;
   ros::Subscriber leftFTSub_;
   ros::Subscriber rightFTSub_;



};

}

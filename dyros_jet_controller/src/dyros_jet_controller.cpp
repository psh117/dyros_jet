/**
  @title DYROS JET Controller
  @authors Jimin Lee, Suhan Park
  */

#include <ros/ros.h>
#include "dyros_jet_controller/simulation_interface.h"
#include "dyros_jet_controller/mujoco_interface.h"
#include "dyros_jet_controller/real_robot_interface.h"
#include "cvxgen_6_8_0/cvxgen/solver.h"
using namespace dyros_jet_controller;

#include <math.h>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "dyros_jet_controller");
    ros::NodeHandle nh("~");

    std::string mode;
    nh.param<std::string>("run_mode", mode, "simulation");
    ControlBase *ctr_obj;

    double Hz;
    nh.param<double>("control_frequency", Hz, 200.0);

    /*
    // FOR DEBUG ATTACHMENT
    char g;
    cout << "press something \n";
    cin >> g;
    */
    if(mode == "simulation")
    {
        ROS_INFO("DYROS JET MAIN CONTROLLER - !!! SIMULATION MODE !!!");
        ctr_obj = new SimulationInterface(nh, Hz);
    }
    else if(mode == "mujoco")
    {
        ROS_INFO("DYROS JET MAIN CONTROLLER - !!! MUJOCO SIMULATION MODE !!!");
        ctr_obj = new mujoco_interface(nh, Hz);
    }
    else if(mode == "real_robot")
    {
        ROS_INFO("DYROS JET MAIN CONTROLLER - !!! REAL ROBOT MODE !!!");
        ctr_obj = new RealRobotInterface(nh, Hz);
        //ROS_ERROR("REAL ROBOT MODE IS NOT IMPLEMENTED YET!!!");
    }
    else
    {
        ROS_FATAL("Please choose simulation or real_robot");
    }

    while(ros::ok())
    {
        ctr_obj->readDevice();
        ctr_obj->update();
        ctr_obj->compute();
        ctr_obj->reflect();
        ctr_obj->writeDevice();
        ctr_obj->wait();
        if(ctr_obj->isShuttingDown())
        {
          break;
        }
    }

    delete ctr_obj;

    return 0;
}


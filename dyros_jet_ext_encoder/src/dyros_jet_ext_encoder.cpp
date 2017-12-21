
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <thread>
#include <string>
#include <vector>
#include <functional>
#include <mutex>
#include <realtime_tools/realtime_publisher.h>


using namespace std;
using namespace boost;


const string JOINT_NAME[40] = {"WaistPitch","WaistYaw",
                               "R_ShoulderPitch","R_ShoulderRoll","R_ShoulderYaw","R_ElbowRoll","R_WristYaw","R_WristRoll","R_HandYaw",
                               "L_ShoulderPitch","L_ShoulderRoll","L_ShoulderYaw","L_ElbowRoll","L_WristYaw","L_WristRoll","L_HandYaw",
                               "R_HipYaw","R_HipRoll","R_HipPitch","R_KneePitch","R_AnklePitch","R_AnkleRoll",
                               "L_HipYaw","L_HipRoll","L_HipPitch","L_KneePitch","L_AnklePitch","L_AnkleRoll",
                               "HeadYaw", "HeadPitch", "R_Gripper", "L_Gripper"};

const int LEFT_LEG_START_NUM = 16;
const int RIGHT_LEG_START_NUM = 22;
const int MAX_JOINT = 32;


/**
 * @brief decoding
 * @param data input array of data after eliminating header
 * @param encoderValue output value of receieved encoder value.
 * @return true - only if checksum is correct. otherwise false.
 */
bool decoding(uint8_t *data, int32_t &encoderValue)
{
  int checkSum = (int)(data[0] + data[1] + data[2] + data[3]);
  int recvCheckSum =
      (data[4] << 8) +
      (data[5]);

  if (checkSum != recvCheckSum)
  {
    return false;
  }
  else
  {
    encoderValue =
      (data[0] << 24) +
      (data[1] << 16) +
      (data[2] << 8) +
      (data[3]);
    return true;
  }
}



void encoder_value_listening_task(string &portName, int32_t &encoderValue, std::mutex &mutex_lock, bool &quit_flag)
{
  try
  {
    ROS_INFO("[%s] Openning Port", portName.c_str());
    asio::io_service io_service;
    asio::serial_port sp(io_service, portName);
    asio::serial_port::baud_rate baud(1000000);
    sp.set_option(baud);

    boost::asio::deadline_timer t(io_service);

    uint8_t rxBuffer[20];
    uint64_t failCount = 0;

    int32_t _encoderValue;
    int headerCount = 0;

    ROS_INFO("[%s] Start Listening", portName.c_str());
    while(!quit_flag)
    {
      asio::read(sp, asio::buffer(rxBuffer, 1));

      //ROS_INFO("read = %d",(int)rxBuffer[0]);
      if (rxBuffer[0] == 0xFF) headerCount ++;
      else headerCount = 0;

      if (headerCount == 2)
      {
        asio::read(sp, asio::buffer(rxBuffer, 6));

        if (decoding(rxBuffer, _encoderValue) == false)
        {
          ROS_WARN("Checksum missmathced");
          failCount ++;
        }
        else
        {
          mutex_lock.lock();
          encoderValue = _encoderValue;
          mutex_lock.unlock();
        }
      }
    }
  }
  catch(std::exception& e)
  {
      ROS_INFO("%s", e.what());
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dyros_jet_ext_encoder");
    ros::NodeHandle nh("~");

    bool quitFlag = false;

    vector<int32_t> encoderValue(MAX_JOINT);
    vector<string> portNames(MAX_JOINT);
    vector<std::mutex> mutexs(MAX_JOINT);


    // get roslaunch param
    for (size_t i=0; i < portNames.size(); i++)
    {
      nh.param<std::string>(JOINT_NAME[i] + "_Port", portNames[i], "None");
    }

    vector<string> messageNames;
    vector<int> messageIndex;

    // threads start
    ROS_INFO("--- Attached Encoders ---");
    vector< std::thread > value_listeners;
    for (size_t i=0; i < portNames.size(); i++)
    {
      if (portNames[i] != "None"){
        ROS_INFO("%s Joint - Port: %s", JOINT_NAME[i].c_str(), portNames[i].c_str());


        value_listeners.push_back(std::thread(encoder_value_listening_task,
                                         std::ref(portNames[i]),
                                         std::ref(encoderValue[i]),
                                         std::ref(mutexs[i]),
                                         std::ref(quitFlag)));

        messageNames.push_back(JOINT_NAME[i]);
        messageIndex.push_back(i);
      }

    }

    // ROS
    ros::Rate r(400);

    realtime_tools::RealtimePublisher<sensor_msgs::JointState> pub(nh, "/dyros_jet/ext_encoder", 4);

    pub.msg_.name = messageNames;
    pub.msg_.position.resize(messageNames.size());

    while(ros::ok())
    {
      for (auto& m : mutexs) { m.lock(); }
      if(pub.trylock())
      {
        for (size_t i=0; i<messageNames.size(); i++)
        {
          pub.msg_.position[i] = (double)encoderValue[messageIndex[i]];
        }
        pub.msg_.header.stamp = ros::Time::now();
        pub.unlockAndPublish();
      }
      for (auto& m : mutexs) { m.unlock(); }
      r.sleep();
    }

    quitFlag = true;

    // quit
    for (size_t i=0; i < value_listeners.size(); i++)
    {
      if (portNames[i] != "None")
      {
        value_listeners[i].join();
      }
    }

    return 0;
}


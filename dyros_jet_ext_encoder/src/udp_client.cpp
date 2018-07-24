
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <thread>
#include <string>
#include <vector>
#include <functional>
#include <mutex>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace boost;

using boost::asio::ip::udp;


const string JOINT_NAME[40] = {"WaistPitch","WaistYaw",
                               "R_ShoulderPitch","R_ShoulderRoll","R_ShoulderYaw","R_ElbowRoll","R_WristYaw","R_WristRoll","R_HandYaw",
                               "L_ShoulderPitch","L_ShoulderRoll","L_ShoulderYaw","L_ElbowRoll","L_WristYaw","L_WristRoll","L_HandYaw",
                               "R_HipYaw","R_HipRoll","R_HipPitch","R_KneePitch","R_AnklePitch","R_AnkleRoll",
                               "L_HipYaw","L_HipRoll","L_HipPitch","L_KneePitch","L_AnklePitch","L_AnkleRoll",
                               "HeadYaw", "HeadPitch", "R_Gripper", "L_Gripper"};

const int LEFT_LEG_START_NUM = 16;
const int RIGHT_LEG_START_NUM = 22;
const int MAX_JOINT = 32;

const double CNT_PER_REV = 983040.0;
const double CNT_PER_DEG = CNT_PER_REV / 360.0;
const double CNT_PER_RAD = CNT_PER_REV / ( M_PI * 2 );
const double RAD_PER_CNT = ( M_PI * 2 ) / CNT_PER_REV;

const double DSP_HZ = 1000.;


ros::Publisher joint_pub;

enum { max_length = 1024 };

uint8_t data[max_length];
uint8_t data2[max_length];
boost::asio::io_service io_service;
udp::socket udp_socket(io_service, udp::endpoint(udp::v4(), 1107));
udp::socket udp_socket2(io_service, udp::endpoint(udp::v4(), 1108));
udp::endpoint sender_endpoint;
udp::endpoint sender_endpoint2;


std::vector<double> q(12);
std::vector<double> qdot(12);

std::mutex data_mutex;

int32_t make_32s(uint8_t *data)
{
  return (data[0] << 8*3) + (data[1] << 8*2) + (data[2] << 8*1) + (data[3] << 8*0);
}
uint32_t make_32u(uint8_t *data)
{
  return (data[0] << 8*3) + (data[1] << 8*2) + (data[2] << 8*1) + (data[3] << 8*0);
}
int16_t make_16s(uint8_t *data)
{
  return (data[0] << 8*1) + (data[1] << 8*0);
}

void handle_receive_from(const boost::system::error_code& error,
    size_t bytes_recvd)
{
  if (!error && bytes_recvd > 50)
  {
    // SEND
    int startIndex = -1;
    if(data[1] == 'L')
    {
      // Leg
      if(data[0] == 'L')
      {
        // Left
        startIndex = 6;
      }
      else if(data[0] == 'R')
      {
        // Right
        startIndex = 0;
      }
    }

    if(startIndex == -1)
    {
      udp_socket.async_receive_from(
          boost::asio::buffer(data, max_length),
            sender_endpoint,handle_receive_from);
      return;
    }

    unsigned int timeStamp = make_32u(&data[2]);
    unsigned int seq = make_32u(&data[6]);
    int length = data[10];


    if(length != 6)
    {
      ROS_WARN("The length of the encoder values is not 6");
    }

    // CHECKSUM!!!!!!!!
    uint32_t checkSum = 0;
    uint32_t checkSumCalc = 0;
    for(int i=0; i<16+(length-1)*6; i++)
    {
        checkSum += data[i];
    }
    checkSumCalc = make_32u(&data[17+(length-1)*6]);
    if(checkSum != checkSumCalc)
    {
        ROS_WARN("Checksum error calc = %x, recv = %x", checkSum, checkSumCalc);
    }
    else
    {
        data_mutex.lock();
        for(int i=0; i<length; i++)
        {
          int q_cnt = make_32s(&data[11 + i * 6]);
          int qdot_cnt = make_16s(&data[15 + i * 6]);
          if (abs(q_cnt*RAD_PER_CNT) > 2.0)
          {
              ROS_WARN("Too much value idx: %d q: %lf", i, q_cnt*RAD_PER_CNT);
              for(int k=0; k<bytes_recvd; k++)
              {
                  printf("%d ", data[k]);
              }
              printf("\n");
          }
          else
          {
              q[startIndex + 5 - i] = q_cnt * RAD_PER_CNT;
              qdot[startIndex + 5 - i] = qdot_cnt * RAD_PER_CNT * DSP_HZ;
          }
        }
        data_mutex.unlock();
    }


  }
  udp_socket.async_receive_from(
      boost::asio::buffer(data, max_length),
        sender_endpoint,handle_receive_from);
}

void handle_receive_from2(const boost::system::error_code& error,
    size_t bytes_recvd)
{
  if (!error && bytes_recvd > 40)
  {
      // Protocol Left/Right Leg/Arm
    // SEND
    int startIndex = -1;
    if(data2[1] == 'L')
    {
      // Leg
      if(data2[0] == 'L')
      {
        // Left
        startIndex = 6;
      }
      else if(data2[0] == 'R')
      {
        // Right
        startIndex = 0;
      }
    }

    if(startIndex == -1)
    {
      udp_socket2.async_receive_from(
          boost::asio::buffer(data2, max_length),
            sender_endpoint2,handle_receive_from2);
      return;
    }

    unsigned int timeStamp = make_32u(&data2[2]);
    unsigned int seq = make_32u(&data2[6]);
    int length = data2[10];


    if(length != 6)
    {
      ROS_WARN("The length of the encoder values is not 6");
    }


    // CHECKSUM!!!!!!!!
    uint32_t checkSum = 0;
    uint32_t checkSumCalc = 0;
    for(int i=0; i<16+(length-1)*6; i++)
    {
        checkSum += data2[i];
    }
    checkSumCalc = make_32u(&data2[17+(length-1)*6]);
    if(checkSum != checkSumCalc)
    {
        ROS_WARN("Checksum error calc = %x, recv = %x", checkSum, checkSumCalc);
    }
    else
    {
        data_mutex.lock();
        for(int i=0; i<length; i++)
        {
          int q_cnt = make_32s(&data2[11 + i * 6]);
          int qdot_cnt = make_16s(&data2[15 + i * 6]);
          if (abs(q_cnt*RAD_PER_CNT) > 2.0)
          {
              ROS_WARN("Too much value idx: %d q: %lf", i, q_cnt*RAD_PER_CNT);
              for(int k=0; k<bytes_recvd; k++)
              {
                  printf("%d ", data2[k]);
              }
              printf("\n");
          }
          else
          {
              q[startIndex + 5 - i] = q_cnt * RAD_PER_CNT;
              qdot[startIndex + 5 - i] = qdot_cnt * RAD_PER_CNT * DSP_HZ;
          }
        }
        data_mutex.unlock();
    }


  }
  udp_socket2.async_receive_from(
      boost::asio::buffer(data2, max_length),
        sender_endpoint2,handle_receive_from2);
}

sensor_msgs::JointState joint_msg;

void joint_publish()
{
  data_mutex.lock();
  joint_msg.position = q;
  joint_msg.velocity = qdot;
  data_mutex.unlock();

  joint_pub.publish(joint_msg);

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "dyros_jet_ext_encoder_udp");
    ros::NodeHandle nh("~");

    joint_pub = nh.advertise<sensor_msgs::JointState>("/dyros_jet/ext_encoder", 5);
    udp_socket.async_receive_from(boost::asio::buffer(data, max_length), sender_endpoint,handle_receive_from);
    udp_socket2.async_receive_from(boost::asio::buffer(data2, max_length), sender_endpoint2,handle_receive_from2);
    boost::asio::io_service::work work(io_service);
    boost::thread thread(boost::bind(&boost::asio::io_service::run, &io_service));

    joint_msg.name.resize(12);
    joint_msg.position.resize(12);
    joint_msg.velocity.resize(12);

    for(int i=0; i<12; i++)
    {
      joint_msg.name[i] = JOINT_NAME[LEFT_LEG_START_NUM + i];
    }

    ros::Rate r(200);
    while(ros::ok())
    {
      joint_publish();
      ros::spinOnce();
      r.sleep();
    }

    return 0;
}


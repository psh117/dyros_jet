#include <dyros_jet_dynamixel_x_interface/dxl_handler.h>

DXLHandler::DXLHandler(const char* port_name, float protocol_version, int baud_rate)
{
  joint_name_map_["hand_finger1"] = 53;
  joint_name_map_["hand_finger2"] = 54;
  joint_name_map_["hand_thumb_fe"] = 55;
  joint_name_map_["hand_thumb_aa"] = 56;

  hand_motor_offset_["hand_finger1"] = 3452;
  hand_motor_offset_["hand_finger2"] = 3481;
  hand_motor_offset_["hand_thumb_fe"] = 2560;
  hand_motor_offset_["hand_thumb_aa"] = 1580;

  
  hand_motor_sign_["hand_finger1"] = -1;
  hand_motor_sign_["hand_finger2"] = -1;
  hand_motor_sign_["hand_thumb_fe"] = -1;
  hand_motor_sign_["hand_thumb_aa"] = +1;

  hand_motor_min_["hand_finger1"] = 0 * M_PI / 180;
  hand_motor_min_["hand_finger2"] = 0 * M_PI / 180;
  hand_motor_min_["hand_thumb_fe"] = 0 * M_PI / 180;
  hand_motor_min_["hand_thumb_aa"] = -15 * M_PI / 180;

  hand_motor_max_["hand_finger1"] = 90 * M_PI / 180;
  hand_motor_max_["hand_finger2"] = 90 * M_PI / 180;
  hand_motor_max_["hand_thumb_fe"] = 90 * M_PI / 180;
  hand_motor_max_["hand_thumb_aa"] = 90 * M_PI / 180;
  
  joint_command_sub_ = nh.subscribe("/dyros_jet/hand_command", 5, &DXLHandler::commandCallback, this);

  joint_pub_ = nh.advertise<sensor_msgs::JointState>("/thormang/head_joint",1);
  joint_msg_.name.push_back("head_lidar");
  joint_msg_.effort.push_back(0.);
  joint_msg_.position.push_back(0.);

  // RPM / 60 * 2 pi
  joint_msg_.velocity.push_back(0.229 * HEAD_LIDAR_VELOCITY / 60. * 2 * M_PI);
  portHandler_ = dynamixel::PortHandler::getPortHandler(port_name);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version);
  open(baud_rate);
}

void DXLHandler::open(int baud)
{
  // Open port
  if (portHandler_->openPort())
  {
    ROS_INFO("THRMNG DXL - Succeeded to open the dynamixel port!");
  }
  else
  {
    ROS_ERROR("THRMNG DXL - Failed to open the port!");
    return ;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baud))
  {
    ROS_INFO("THRMNG DXL - Succeeded to change the baudrate!");
  }
  else
  {
    ROS_ERROR("THRMNG DXL - Failed to change the baudrate!");
    return ;
  }
}

void DXLHandler::setTorqueX(int id, int enable)
{
  comm_result_ = packetHandler_->write1ByteTxRx(portHandler_,
                                                id,
                                                ADDR_X_TORQUE_ENABLE,
                                                enable,
                                                &error_);
}
void DXLHandler::setTorquePro(int id, int enable)
{
  comm_result_ = packetHandler_->write1ByteTxRx(portHandler_,
                                                id,
                                                ADDR_PRO_TORQUE_ENABLE,
                                                enable,
                                                &error_);
}
void DXLHandler::setPositionPro(int id, int position)
{
  comm_result_ = packetHandler_->write4ByteTxRx(portHandler_,
                                                id,
                                                ADDR_PRO_GOAL_POSITION,
                                                position,
                                                &error_);
}
void DXLHandler::setPositionX(int id, int position)
{
  comm_result_ = packetHandler_->write4ByteTxRx(portHandler_,
                                                id,
                                                ADDR_X_GOAL_POSITION,
                                                position,
                                                &error_);
}
void DXLHandler::setVelocityX(int id, int velocity)
{
  comm_result_ = packetHandler_->write4ByteTxRx(portHandler_,
                                                id,
                                                ADDR_X_GOAL_VELOCITY,
                                                velocity,
                                                &error_);
}

void DXLHandler::allTorqueOn()
{

  ROS_INFO("torque ON!");
  setTorqueX(49, 1);
  setTorquePro(51, 1);
  setTorquePro(52, 1);

  setTorqueX(53, 1);
  setTorqueX(54, 1);
  setTorqueX(55, 1);
  setTorqueX(56, 1);
}

void DXLHandler::allTorqueOff()
{
  ROS_INFO("torque OFF!!!!!");
  setTorqueX(49, 0);
  setTorquePro(51, 0);
  setTorquePro(52, 0);

  setTorqueX(53, 0);
  setTorqueX(54, 0);
  setTorqueX(55, 0);
  setTorqueX(56, 0);
}

void DXLHandler::setInitCommand()
{
  ROS_INFO("set pos / vel!");
  setVelocityX(49, HEAD_LIDAR_VELOCITY);
  setPositionPro(51, 0);
  setPositionPro(52, -17000);
}

void DXLHandler::controlLoop()
{
  ros::Rate r(125);
  while(ros::ok())
  {
    int32_t present_position;
    comm_result_ = packetHandler_->read4ByteTxRx(portHandler_, 49, ADDR_X_PRESENT_POSITION,
                                                 (uint32_t*)&present_position, &error_);
    if(comm_result_ == COMM_SUCCESS)
    {
      joint_msg_.header.stamp = ros::Time::now();
      //ROS_INFO("%d",present_position);
      presentPosition_ = ((present_position % 4096)-2048) * 0.001533981;
      joint_msg_.position[0] = presentPosition_;
      joint_pub_.publish(joint_msg_);
    }
    else
    {
      ROS_WARN("COMM FAILED, %d",comm_result_);
    }

    ros::spinOnce();
    r.sleep();
  }
}

void DXLHandler::commandCallback(const sensor_msgs::JointState::ConstPtr & msg)
{
  const int n = msg->name.size();
  for(int i=0; i<n; i++)
  {
    if(joint_name_map_.find(msg->name[i]) == joint_name_map_.end() )  // If the name does not exist
    {
      ROS_WARN("I got a wrong name :%s", msg->name[i].c_str());
      continue;
    }
    double target_pos = hand_motor_offset_[msg->name[i]] +
        max(hand_motor_min_[msg->name[i]],
        min(hand_motor_max_[msg->name[i]], msg->position[i])
        ) * hand_motor_sign_[msg->name[i]] * 2048./M_PI;

    setPositionX(joint_name_map_[msg->name[i]],target_pos);

  }
}

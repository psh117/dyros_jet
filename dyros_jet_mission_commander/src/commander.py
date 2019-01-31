#!/usr/bin/env python
import rospy
from dyros_jet_msgs.msg import JointCommand
from dyros_jet_msgs.msg import TaskCommand
from dyros_jet_msgs.msg import WalkingCommand
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

def commander():
    pub = rospy.Publisher('/dyros_jet/joint_command', JointCommand, queue_size=10)
    task_pub = rospy.Publisher('/dyros_jet/task_command', TaskCommand, queue_size=10)
    walk_pub = rospy.Publisher('/dyros_jet/walking_command',WalkingCommand, queue_size =10)
    rospy.init_node('mission_commander', anonymous=True)
    r = rospy.Rate(1) #50hz
#    r.sleep()

    print('waiting for connection')
    while not rospy.is_shutdown() and pub.get_num_connections() >= 1:
        rospy.sleep(.5)
        pass
    print('connected')
    print('waiting for connection2')
    while not rospy.is_shutdown() and task_pub.get_num_connections() >= 1:
        rospy.sleep(.5)
        pass
    print('connected')
    print('waiting for connection3')
    while not rospy.is_shutdown() and walk_pub.get_num_connections() >= 1:
        rospy.sleep(.5)
        pass
    print('connected')

    rospy.sleep(2.)

    msg = JointCommand()
    msg.name = ["L_HipYaw","L_HipRoll","L_HipPitch","L_KneePitch","L_AnklePitch","L_AnkleRoll","R_HipYaw","R_HipRoll","R_HipPitch","R_KneePitch","R_AnklePitch","R_AnkleRoll","WaistPitch","WaistYaw", "L_ShoulderPitch","L_ShoulderRoll","L_ShoulderYaw","L_ElbowRoll","L_WristYaw","L_WristRoll","L_HandYaw", "R_ShoulderPitch","R_ShoulderRoll","R_ShoulderYaw","R_ElbowRoll","R_WristYaw","R_WristRoll","R_HandYaw","HeadYaw", "HeadPitch", "R_Gripper", "L_Gripper"]
    #msg.enable = [True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, False, False, False, False]

    msg.position = [0 , 0.034906585 , -0.3490658504 , 0.6981317008 , -0.3490658504 , -0.034906585 , 0 , -0.034906585 , 0.3490658504 , -0.6981317008 , 0.3490658504 , 0.034906585 , 0 , 0 , 0.6981317008 , -1.6580627893 , -1.3962634016 , -1.9198621771 , 0 , -1.2217304764 , -0.1745329252 , -0.6981317008 , 1.6580627893 , 1.3962634016 , 1.9198621771 , 0 , 1.2217304764 , 0.17453292519 , 0 , 0 , 0 , 0 ]
    msg.duration = [5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 0 , 0 , 0 , 0]
    pub.publish(msg)
    time = 0
    rospy.sleep(5.)


    #task_msg = TaskCommand()
    #
    #
    #task_msg.end_effector=[False, False, False, True]
    #task_msg.mode=[0, 0, 0, 0]
    #task_msg.pose[3].position = Point(0.3, 0.0, 0.0)
    #task_msg.pose[3].orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    #task_msg.duration = [0.0, 0.0, 0.0, 5.0]
    #task_pub.publish(task_msg);
    #
    #
    #rospy.sleep(5.0);
    #
    #task_msg.end_effector=[True, False, False, False]
    #task_msg.mode=[0, 0, 0, 0]
    #task_msg.pose[0].position = Point(0.0, 0.0, 0.2)
    #task_msg.pose[0].orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    #task_msg.duration = [5.0, 0.0, 0.0, 0.0]
    #task_pub.publish(task_msg);
    #
    #rospy.sleep(5.0);
    
    msg = JointCommand()
    msg.name = ["L_HipYaw","L_HipRoll","L_HipPitch","L_KneePitch","L_AnklePitch","L_AnkleRoll","R_HipYaw","R_HipRoll","R_HipPitch","R_KneePitch","R_AnklePitch","R_AnkleRoll","WaistPitch","WaistYaw", "L_ShoulderPitch","L_ShoulderRoll","L_ShoulderYaw","L_ElbowRoll","L_WristYaw","L_WristRoll","L_HandYaw", "R_ShoulderPitch","R_ShoulderRoll","R_ShoulderYaw","R_ElbowRoll","R_WristYaw","R_WristRoll","R_HandYaw","HeadYaw", "HeadPitch", "R_Gripper", "L_Gripper"]
    msg.position = [0 , 0.034906585 , -0.034906585 , 0.733038286 , -0.698131701 , -0.034906585 , 0 , -0.034906585 , 0.034906585 , -0.733038286 , 0.698131701 , 0.034906585 , 0 , 0 , 0.6981317008 , -1.6580627893 , -1.3962634016 , -1.9198621771 , 0 , -1.2217304764 , -0.1745329252 , -0.6981317008 , 1.6580627893 , 1.3962634016 , 1.9198621771 , 0 , 1.2217304764 , 0.17453292519 , 0 , 0 , 0 , 0 ]
    msg.duration = [5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 0 , 0 , 0 , 0]

    rospy.sleep(0.5)
    pub.publish(msg)
    time = 0
    rospy.sleep(15.)


    walk_msg= WalkingCommand()

    walk_msg.walk_mode = 1
    walk_msg.compensator_mode = [False, False]
    walk_msg.ik_mode = 1
    walk_msg.first_foot_step = False
    walk_msg.heel_toe = False
    walk_msg.x = 0.0
    walk_msg.y = 0.0
    walk_msg.z = 0.0
    walk_msg.height = 0.75
    walk_msg.theta = 0
    walk_msg.step_length_x =0.20
    walk_msg.step_length_y =0.10


    walk_pub.publish(walk_msg);


if __name__ == '__main__':
    try:
        commander()
    except rospy.ROSInterruptException: pass

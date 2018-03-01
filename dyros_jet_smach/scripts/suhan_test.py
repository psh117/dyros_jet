#!/usr/bin/env python

import roslib; roslib.load_manifest('dyros_jet_smach')
import rospy
from smach import StateMachine
import smach_ros

from dyros_jet_msgs.msg import JointControlAction, JointControlGoal
from actionlib import *
from actionlib_msgs.msg import *
from smach_ros import SimpleActionState

def main():
    rospy.init_node('suhan_test_smach')


    #sm = StateMachine(['succeeded','aborted','preempted'])
    sm = StateMachine(outcomes=['READY_FOR_ORDER','preempted', 'aborted'])
    with sm:
        joint_goal = JointControlGoal()

        joint_goal.command.name = ["L_HipYaw","L_HipRoll","L_HipPitch","L_KneePitch","L_AnklePitch","L_AnkleRoll","R_HipYaw","R_HipRoll","R_HipPitch","R_KneePitch","R_AnklePitch","R_AnkleRoll","WaistPitch","WaistYaw", "L_ShoulderPitch","L_ShoulderRoll","L_ShoulderYaw","L_ElbowRoll","L_WristYaw","L_WristRoll","L_HandYaw", "R_ShoulderPitch","R_ShoulderRoll","R_ShoulderYaw","R_ElbowRoll","R_WristYaw","R_WristRoll","R_HandYaw","HeadYaw", "HeadPitch", "R_Gripper", "L_Gripper"]
        #msg.enable = [True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, False, False, False, False]
        joint_goal.command.position = [0 , 0.034906585 , -0.3490658504 , 0.6981317008 , -0.3490658504 , -0.034906585 , 0 , -0.034906585 , 0.3490658504 , -0.6981317008 , 0.3490658504 , 0.034906585 , 0 , 0 , 0.6981317008 , -1.6580627893 , -1.3962634016 , -1.9198621771 , 0 , -1.2217304764 , -0.1745329252 , -0.6981317008 , 1.6580627893 , 1.3962634016 , 1.9198621771 , 0 , 1.2217304764 , 1.7453292519 , 0 , 0 , 0 , 0 ]
        joint_goal.command.duration = [5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 0 , 0 , 0 , 0]

        StateMachine.add('FOO',
        SimpleActionState('/dyros_jet/joint_control', JointControlAction, goal=joint_goal),
        transitions={'succeeded':'READY_FOR_ORDER'})



    # Execute SMACH plan
    outcome = sm.execute()

    rospy.signal_shutdown('All done.')


if __name__ == '__main__':
    main()

#!/usr/bin/env python

import roslib; roslib.load_manifest('dyros_jet_smach')
import rospy
from smach import StateMachine
import smach_ros
import smach

from dyros_jet_msgs.msg import JointControlAction, JointControlGoal
import rt_dynamixel_msgs.srv
from actionlib import *
from actionlib_msgs.msg import *
from smach_ros import SimpleActionState
from smach_ros import ServiceState
from std_msgs.msg import String


class StringTransitionState(smach.State):
    topic=''
    def __init__(self, topic, outcomes=[], input_keys=[], output_keys=[]):
        self._topic = topic
        smach.State.__init__(self, outcomes, input_keys, output_keys)

    def execute(self, userdata):
        print(self._topic)
        while True:
            print('wait for message')
            trans_tag = rospy.wait_for_message(self._topic,String)
            print(trans_tag.data)

            if trans_tag.data in self._outcomes:
                return trans_tag.data



def main():
    rospy.init_node('mini_drc_toplevel')

    topic_name = '/dyros_jet/smach/transition'

    joint_init_goal = JointControlGoal()

    joint_init_goal.command.name = ['L_HipYaw','L_HipRoll','L_HipPitch','L_KneePitch','L_AnklePitch','L_AnkleRoll','R_HipYaw','R_HipRoll','R_HipPitch','R_KneePitch','R_AnklePitch','R_AnkleRoll','WaistPitch','WaistYaw', 'L_ShoulderPitch','L_ShoulderRoll','L_ShoulderYaw','L_ElbowRoll','L_WristYaw','L_WristRoll','L_HandYaw', 'R_ShoulderPitch','R_ShoulderRoll','R_ShoulderYaw','R_ElbowRoll','R_WristYaw','R_WristRoll','R_HandYaw','HeadYaw', 'HeadPitch', 'R_Gripper', 'L_Gripper']
    #msg.enable = [True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, False, False, False, False]
    joint_init_goal.command.position = [0 , 0.034906585 , -0.034906585 , 0.733038285 , -0.6981317 , -0.034906585, 0 , -0.034906585 , 0.0349065850 , -0.733038285 , 0.6981317 , 0.034906585, 0 , 0, 0.6981317008 , -1.6580627893 , -1.3962634016 , -1.9198621771 , 0 , -1.2217304764 , -0.1745329252, -0.6981317008 , 1.6580627893 , 1.3962634016 , 1.9198621771 , 0 , 1.2217304764 , 0.17453292519, 0 , 0 , 0 , 0]
    joint_init_goal.command.duration = [3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 3 , 0 , 0 , 0 , 0]


    # Construct state machine
    mini_drc_sm = StateMachine(
            outcomes=['finished','aborted','preempted'])

    #run_mode = rospy.get_param('~run_mode', 'simulation')
    run_mode = rospy.get_param('~run_mode', 'real_robot')
    print(run_mode)
    # Set the initial state explicitly
    if run_mode == 'simulation':
        mini_drc_sm.set_initial_state(['READY'])
    elif run_mode == 'mujoco':
          mini_drc_sm.set_initial_state(['READY'])
    elif run_mode == 'real_robot':
        mini_drc_sm.set_initial_state(['POWER_OFF'])
    else:
        print 'unknown mode'

    #request = rt_dynamixel_msgs.srv.ModeSettingRequest(rt_dynamixel_msgs.srv.ModeSettingRequest.SETTING)
    #print(request)
    #request = rt_dynamixel_msgs.srv.MotorSettingRequest(
    #mode=rt_dynamixel_msgs.srv.MotorSettingRequest.SET_TORQUE_ENABLE,value=1)
    #print(request)

    with mini_drc_sm:
        StateMachine.add('POWER_OFF',
        StringTransitionState(topic_name, outcomes=['power_on']),
        {'power_on':'SET_DXL_MODE_SETTING_MODE'})

        StateMachine.add('SET_DXL_MODE_SETTING_MODE',
            ServiceState('/rt_dynamixel/mode', rt_dynamixel_msgs.srv.ModeSetting,
            request = rt_dynamixel_msgs.srv.ModeSettingRequest(
            rt_dynamixel_msgs.srv.ModeSettingRequest.SETTING)),
            transitions={'succeeded':'SET_DXL_TORQUE_ON', 'aborted':'SET_DXL_MODE_SETTING_MODE'})

        StateMachine.add('SET_DXL_TORQUE_ON',
            ServiceState('/rt_dynamixel/motor_set', rt_dynamixel_msgs.srv.MotorSetting,
            request = rt_dynamixel_msgs.srv.MotorSettingRequest(
            mode=rt_dynamixel_msgs.srv.MotorSettingRequest.SET_TORQUE_ENABLE,value=1)),
            transitions={'succeeded':'SET_DXL_SYNC_DRIVE_ON', 'aborted':'SET_DXL_MODE_SETTING_MODE'})

        StateMachine.add('SET_DXL_SYNC_DRIVE_ON',
            ServiceState('/rt_dynamixel/mode', rt_dynamixel_msgs.srv.ModeSetting,
            request = rt_dynamixel_msgs.srv.ModeSettingRequest(
            rt_dynamixel_msgs.srv.ModeSettingRequest.CONTROL_RUN)),
            transitions={'succeeded':'READY', 'aborted':'SET_DXL_MODE_SETTING_MODE'})

        StateMachine.add('READY',
            StringTransitionState(topic_name, outcomes=['initialize_pose']),
            transitions={'initialize_pose':'SET_INIT_POSITION'})

        StateMachine.add('SET_INIT_POSITION',
            SimpleActionState('/dyros_jet/joint_control', JointControlAction, goal=joint_init_goal),
            transitions={'succeeded':'READY_TO_MOVE'})

        StateMachine.add('READY_TO_MOVE',
            StringTransitionState(topic_name, outcomes=['Mot1']),
            transitions={'Mot1':'Motion1'})

        StateMachine.add('Motion1',
            StringTransitionState(topic_name, outcomes=['Mot2']),
            transitions={'Mot2':'Motion2'})

        StateMachine.add('Motion2',
            StringTransitionState(topic_name, outcomes=['stair', 'door', 'initialize_pose1']),
            transitions={'stair':'finished', 'door':'finished', 'initialize_pose1':'SET_INIT_POSITION'})

    # Run state machine introspection server
    intro_server = smach_ros.IntrospectionServer('dyros_jet',mini_drc_sm,'/MINI_DRC')
    intro_server.start()
    mini_drc_sm.execute()

    rospy.spin()

    intro_server.stop()


    rospy.signal_shutdown('All done.')


if __name__ == '__main__':
    main()

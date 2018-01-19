#!/usr/bin/env python

import roslib; roslib.load_manifest('dyros_jet_smach')

import rospy
import smach
import std_srvs.srv
import smach_ros
from smach_ros import ServiceState, SimpleActionState
from turtle_actionlib.msg import ShapeAction, ShapeGoal
from smach import Concurrence

def main():
	rospy.init_node('smach_mini_drc_missions')
	
	sm_root = smach.StateMachine.add('RESET', ServiceState('reset', std_srvs.srv.Empty), {})
	sis = smach_ros.IntrospectionServer('intro_server', sm_root, '/Intro')
	sis.start()

	rospy.spin()
	sis.stop()

	outcome = sm_root.execute()

if __name__ == '__main__':
	main()

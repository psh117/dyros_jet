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
    msg = JointCommand()
    msg.enable = [True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, False, False]
    msg.position = [0 , 0.034906585 , -0.3490658504 , 0.6981317008 , -0.3490658504 , -0.034906585 , 0 , -0.034906585 , 0.3490658504 , -0.6981317008 , 0.3490658504 , 0.034906585 , 0 , 0 , 0.6981317008 , -1.6580627893 , -1.3962634016 , -1.9198621771 , 0 , -1.2217304764 , -0.1745329252 , -0.6981317008 , 1.6580627893 , 1.3962634016 , 1.9198621771 , 0 , 1.2217304764 , 1.7453292519 , 0 , 0 , 0 , 0 ]
    msg.duration = [5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 5 , 0 , 0 , 0 , 0]
    
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
    
    rospy.sleep(0.5)
    pub.publish(msg)
    time = 0
    rospy.sleep(5.)
    
    msg.enable = [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False]
    pub.publish(msg)
    rospy.sleep(.5)

    task_msg = TaskCommand()
    task_msg.end_effector=[False, False, False, True]
    task_msg.mode=[0, 0, 0, 0]
    task_msg.pose[3].position = Point(0.1, 0.0, 0.0)
    task_msg.pose[3].orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    task_msg.duration = [0.0, 0.0, 0.0, 5.0]
    task_pub.publish(task_msg);

    rospy.sleep(5.0);

    walk_msg= WalkingCommand()
    walk_msg.walk_mode = true
    walk_msg.walk_enable = [True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, False, False]

    walk_pub.publish(walk.msg);

    rospy.sleep(5.0);

if __name__ == '__main__':
    try:
        commander()
    except rospy.ROSInterruptException: pass

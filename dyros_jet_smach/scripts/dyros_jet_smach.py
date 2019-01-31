#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from std_msgs.msg import String

class Simple_State(smach.State):
  def execute(self, userdata):
    while True:
      trans_tag = rospy.wait_for_message('/dyros_jet/smach/transition',String)

      if trans_tag.data in self._outcomes:
        return trans_tag.data

#define state Stand_By
class Stand_By(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['power_on','shutdown'])

#define state Power_On
class Power_On(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['auto_on','manu_on','event_on','shutdown'])


#define state Mode_Chg
class Mode_Chg(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['manu_on','auto_on','event_on','shutdown'])


#define state Auto
class Auto(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['mission1','mission2','mission3','mission4','shutdown'])

#define state Manual
class Manual(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['activate_jctrl','activate_tctrl','activate_recog','shutdown'])


class Event(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['handclap','handshake','hello','cmd_modechg','shutdown'])

# Event - Handclap Parts
class Handclap_Start(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['handclap_ready','cmd_modechg','shutdown'])


class Handclap_Ready(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['handclap_do','cmd_modechg','shutdown'])


class Handclap_Do(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['handclap_ready','handclap_end','cmd_modechg','shutdown'])


class Handclap_End(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['handclap_ready','cmd_modechg','shutdown'])



# Event - Handshake Parts
class Handshake_Start(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['handshake_turn','cmd_modechg','shutdown'])
class Handshake_Turn(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['handshake_ready','cmd_modechg','shutdown'])

class Handshake_Ready(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['handshake_do','cmd_modechg','shutdown'])


class Handshake_Do(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['handshake_end','handshake_ready','cmd_modechg','shutdown'])


class Handshake_End(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['handshake_ready','handshake_motion1','handshake_motion2','cmd_modechg','shutdown'])

class Handshake_Motion1(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['handshake_ready','handshake_motion2','cmd_modechg','shutdown'])

class Handshake_Motion2(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['handshake_ready','handshake_motion3','cmd_modechg','shutdown'])

class Handshake_Motion3(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['handshake_ready','handshake_return','cmd_modechg','shutdown'])

class Handshake_Return(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['handshake_ready','cmd_modechg','shutdown'])


# Event - Hello Parts
class Hello_Start(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['hello_ready','hello_introduce','hello_introduce_end','cmd_modechg','shutdown'])

class Hello_Ready(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['hello_do','hello_introduce','hello_introduce_end','cmd_modechg','shutdown'])

class Hello_Do(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['hello_end','hello_ready','hello_introduce','hello_introduce_end','cmd_modechg','shutdown'])

class Hello_End(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['hello_ready','hello_introduce','hello_introduce_end','cmd_modechg','shutdown'])

class Hello_Introduce(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['hello_ready','hello_do','hello_end','hello_introduce_end','cmd_modechg','shutdown'])

class Hello_Introduce_End(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['hello_ready','hello_do','hello_end','hello_introduce','cmd_modechg','shutdown'])


#define state JointCtrl
class JointCtrl(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['jctrl_back','cmd_modechg','shutdown'])

#define state task
class TaskCtrl(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['tctrl_back','cmd_modechg','shutdown'])


class Recog(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['recog_back','cmd_modechg','shutdown'])

#define state Valve_Mission
class Valve_Mission(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['v_init','v_ready','v_approach','v_reach','v_close','shutdown'])


#define state Valve_Init
class Valve_Init(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['v_ready','v_approach','cmd_modechg','shutdown'])

#define state Valve_Ready
class Valve_Ready(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['v_reach','cmd_modechg','shutdown'])

#define state Valve_Reach
class Valve_Reach(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['v_close','cmd_modechg','shutdown'])

#define state Valve_Close
class Valve_Close(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['v_init','cmd_modechg','shutdown'])

#define State Valve_Approach
class Valve_Approach(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['v_init','cmd_modechg','shutdown'])

#define State Door_Mission
class Door_Mission(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['d_init','d_ready','d_reach','d_open','d_push','shutdown'])

#define State Door_Init
class Door_Init(Simple_State):
  def __init__(self):
    smach.State.__init__(self,outcomes=['d_ready','cmd_modechg','shutdown'])


#define State Door_Ready
class Door_Ready(Simple_State):
  def __init__(self):
    smach.State.__init__(self,outcomes=['d_reach','cmd_modechg','shutdown'])

#define State Door_Reach
class Door_Reach(Simple_State):
  def __init__(self):
    smach.State.__init__(self,outcomes=['d_open','cmd_modechg','shutdown'])

#define State Door_Open
class Door_Open(Simple_State):
  def __init__(self):
    smach.State.__init__(self,outcomes=['d_push','cmd_modechg','shutdown'])

#define State Door_Push
class Door_Push(Simple_State):
  def __init__(self):
    smach.State.__init__(self,outcomes=['cmd_dinit','cmd_modechg','shutdown'])


# Egress!

class Egress_Mission(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['e_init','cmd_modechg','shutdown'])

class Egress_Init(Simple_State):
  def __init__(self):
      smach.State.__init__(self, outcomes=['e_egress','cmd_modechg','shutdown'])

class Egress_Egress(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['e_standby','cmd_modechg','shutdown'])


class Egress_Standby(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['e_hello','e_guide','cmd_modechg','shutdown'])

class Egress_Hello(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['e_guide','cmd_modechg','shutdown'])

class Egress_Guide(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['e_hello','cmd_modechg','shutdown'])



#define State Wall_Mission
class Wall_Mission(Simple_State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['w_init','w_approach','w_ready','w_reach','w_grab','w_ungrab','w_rotatedrill','w_wallready','w_contact','w_cut','w_push','shutdown'])


#define State Wall_Init
class Wall_Init(Simple_State):
  def __init__(self):
    smach.State.__init__(self,outcomes=['cmd_wready','cmd_wapproach','cmd_modechg','shutdown'])

#define State Wall_Approach
class Wall_Approach(Simple_State):
  def __init__(self):
    smach.State.__init__(self,outcomes=['cmd_winit','cmd_modechg','shutdown'])


#define State Wall_Ready
class Wall_Ready(Simple_State):
  def __init__(self):
    smach.State.__init__(self,outcomes=['cmd_wreach','cmd_modechg','shutdown'])


#define State Wall_Reach
class Wall_Reach(Simple_State):
  def __init__(self):
    smach.State.__init__(self,outcomes=['cmd_wgrab','cmd_modechg','shutdown'])

#define State Wall_Grab
class Wall_Grab(Simple_State):
  def __init__(self):
    smach.State.__init__(self,outcomes=['cmd_wungrab','cmd_wroratedrill','cmd_wwallready','cmd_modechg','shutdown'])

#define State Wall_Ungrab
class Wall_Ungrab(Simple_State):
  def __init__(self):
    smach.State.__init__(self,outcomes=['cmd_wgrab','cmd_wroratedrill','cmd_modechg','shutdown'])

#define State Wall_Rotatedrill
class Wall_Rotatedrill(Simple_State):
  def __init__(self):
    smach.State.__init__(self,outcomes=['cmd_wgrab','cmd_wungrab','cmd_modechg','shutdown'])

#define State Wall_wallready
class Wall_Wallready(Simple_State):
  def __init__(self):
    smach.State.__init__(self,outcomes=['cmd_wcontact','cmd_modechg','shutdown'])

#define State Wall_Contact
class Wall_Contact(Simple_State):
  def __init__(self):
    smach.State.__init__(self,outcomes=['cmd_wcut','cmd_modechg','shutdown'])

#define State Wall_Cut
class Wall_Cut(Simple_State):
  def __init__(self):
    smach.State.__init__(self,outcomes=['cmd_wpush','cmd_modechg','shutdown'])

#define State Wall_Push
class Wall_Push(Simple_State):
  def __init__(self):
    smach.State.__init__(self,outcomes=['cmd_winit','cmd_modechg','shutdown'])


def main():
  rospy.init_node('dyros_jet_smach')

  # Create a SMACH state machine
  sm_top = smach.StateMachine(outcomes=['END','Wall','Joint','Task','Recognize'])
#    rospy.Subscriber("/transition",String,cb)

  # Open the container
  with sm_top:
    # Stand By state
    smach.StateMachine.add('Stand_By',Stand_By(),
                           transitions={'power_on':'Power_On','shutdown':'END'})

    # Power on state
    smach.StateMachine.add('Power_On',Power_On(),
                           transitions={'auto_on':'Auto','manu_on':'Manual','event_on':'Event','shutdown':'END'})
    # Auto state
    smach.StateMachine.add('Auto',Auto(),
                           transitions={'mission1':'Valve_Mission','mission2':'Door_Mission','mission3':'Egress_Mission','mission4':'Wall','shutdown':'END'})
    # Manual state
    smach.StateMachine.add('Manual',Manual(),
                           transitions={'activate_jctrl':'JointCtrl','activate_tctrl':'TaskCtrl','activate_recog':'Recog','shutdown':'END'})

    # Event state
    smach.StateMachine.add('Event',Event(),
                          transitions={'handclap':'Handclap_Start','handshake':'Handshake_Start', 'hello':'Hello_Start',  'cmd_modechg':'Mode_Chg','shutdown':'END'})
    # JointCtrl state
    smach.StateMachine.add('JointCtrl',JointCtrl(),
                           transitions={'jctrl_back':'Manual','cmd_modechg':'Mode_Chg','shutdown':'END'})
    # TaskCtrl state
    smach.StateMachine.add('TaskCtrl',TaskCtrl(),
                           transitions={'tctrl_back':'Manual','cmd_modechg':'Mode_Chg','shutdown':'END'})
    # Recog state
    smach.StateMachine.add('Recog',Recog(),
                           transitions={'recog_back':'Manual','cmd_modechg':'Mode_Chg','shutdown':'END'})

    # Mode_Chg state
    smach.StateMachine.add('Mode_Chg',Mode_Chg(),transitions={'manu_on':'Manual','auto_on':'Auto','event_on':'Event','shutdown':'END'})

    smach.StateMachine.add('Handclap_Start', Handclap_Start(), transitions={'handclap_ready':'Handclap_Ready','cmd_modechg':'Mode_Chg','shutdown':'END'})
    smach.StateMachine.add('Handclap_Ready', Handclap_Ready(), transitions={'handclap_do':'Handclap_Do','cmd_modechg':'Mode_Chg','shutdown':'END'})
    smach.StateMachine.add('Handclap_Do', Handclap_Do(), transitions={'handclap_ready':'Handclap_Ready','handclap_end':'Handclap_End','cmd_modechg':'Mode_Chg','shutdown':'END'})
    smach.StateMachine.add('Handclap_End', Handclap_End(), transitions={'handclap_ready':'Handclap_Ready','cmd_modechg':'Mode_Chg','shutdown':'END'})

    smach.StateMachine.add('Handshake_Start', Handshake_Start(), transitions={'handshake_turn':'Handshake_Turn','cmd_modechg':'Mode_Chg','shutdown':'END'})
    smach.StateMachine.add('Handshake_Turn', Handshake_Turn(), transitions={'handshake_ready':'Handshake_Ready','cmd_modechg':'Mode_Chg','shutdown':'END'})
    smach.StateMachine.add('Handshake_Ready', Handshake_Ready(), transitions={'handshake_do':'Handshake_Do','cmd_modechg':'Mode_Chg','shutdown':'END'})
    smach.StateMachine.add('Handshake_Do', Handshake_Do(), transitions={'handshake_ready':'Handshake_Ready','handshake_end':'Handshake_End','cmd_modechg':'Mode_Chg','shutdown':'END'})
    smach.StateMachine.add('Handshake_End', Handshake_End(), transitions={'handshake_ready':'Handshake_Ready','handshake_motion1':'Handshake_Motion1','handshake_motion2':'Handshake_Motion2','cmd_modechg':'Mode_Chg','shutdown':'END'})

    smach.StateMachine.add('Handshake_Motion1', Handshake_Motion1(), transitions={'handshake_ready':'Handshake_Ready','handshake_motion2':'Handshake_Motion2','cmd_modechg':'Mode_Chg','shutdown':'END'})
    smach.StateMachine.add('Handshake_Motion2', Handshake_Motion2(), transitions={'handshake_ready':'Handshake_Ready','handshake_motion3':'Handshake_Motion3','cmd_modechg':'Mode_Chg','shutdown':'END'})
    smach.StateMachine.add('Handshake_Motion3', Handshake_Motion3(), transitions={'handshake_ready':'Handshake_Ready','handshake_return':'Handshake_Return','cmd_modechg':'Mode_Chg','shutdown':'END'})
    smach.StateMachine.add('Handshake_Return', Handshake_Return(), transitions={'handshake_ready':'Handshake_Ready','cmd_modechg':'Mode_Chg','shutdown':'END'})


    smach.StateMachine.add('Hello_Start', Hello_Start(), transitions={'hello_ready':'Hello_Ready','hello_introduce':'Hello_Introduce','hello_introduce_end':'Hello_Introduce_End','cmd_modechg':'Mode_Chg','shutdown':'END'})
    smach.StateMachine.add('Hello_Ready', Hello_Ready(), transitions={'hello_do':'Hello_Do','hello_introduce':'Hello_Introduce','hello_introduce_end':'Hello_Introduce_End','cmd_modechg':'Mode_Chg','shutdown':'END'})
    smach.StateMachine.add('Hello_Do', Hello_Do(), transitions={'hello_ready':'Hello_Ready','hello_end':'Hello_End','hello_introduce':'Hello_Introduce','hello_introduce_end':'Hello_Introduce_End','cmd_modechg':'Mode_Chg','shutdown':'END'})
    smach.StateMachine.add('Hello_End', Hello_End(), transitions={'hello_ready':'Hello_Ready','hello_introduce':'Hello_Introduce','hello_introduce_end':'Hello_Introduce_End','cmd_modechg':'Mode_Chg','shutdown':'END'})
    smach.StateMachine.add('Hello_Introduce', Hello_Introduce(), transitions={'hello_ready':'Hello_Ready','hello_do':'Hello_Do','hello_end':'Hello_End','hello_introduce_end':'Hello_Introduce_End','cmd_modechg':'Mode_Chg','shutdown':'END'})
    smach.StateMachine.add('Hello_Introduce_End', Hello_Introduce_End(), transitions={'hello_ready':'Hello_Ready','hello_do':'Hello_Do','hello_end':'Hello_End','hello_introduce':'Hello_Introduce','cmd_modechg':'Mode_Chg','shutdown':'END'})
    # Valve_Mission state
    smach.StateMachine.add('Valve_Mission', Valve_Mission(), transitions={'v_init':'Valve_Init','v_ready':'Valve_Ready','v_approach':'Valve_Approach','v_reach':'Valve_Reach','v_close':'Valve_Close','shutdown':'END'})
    # Valve_init state
    smach.StateMachine.add('Valve_Init', Valve_Init(), transitions={'v_ready':'Valve_Ready','v_approach':'Valve_Approach','cmd_modechg':'Mode_Chg','shutdown':'END'})
    # Valve_ready state
    smach.StateMachine.add('Valve_Ready', Valve_Ready(), transitions={'v_reach':'Valve_Reach','cmd_modechg':'Mode_Chg','shutdown':'END'})
    # Valve_reach state
    smach.StateMachine.add('Valve_Reach', Valve_Reach(), transitions={'v_close':'Valve_Close','cmd_modechg':'Mode_Chg','shutdown':'END'})
    # Valve_close state
    smach.StateMachine.add('Valve_Close', Valve_Close(), transitions={'v_init':'Valve_Init','cmd_modechg':'Mode_Chg','shutdown':'END'})
    # Valve_approach state
    smach.StateMachine.add('Valve_Approach', Valve_Approach(), transitions={'v_init':'Valve_Init','cmd_modechg':'Mode_Chg','shutdown':'END'})

    #Door_Mission state
    smach.StateMachine.add('Door_Mission',Door_Mission(),transitions={'d_init':'Door_Init','d_ready':'Door_Ready','d_reach':'Door_Reach','d_open':'Door_Open','d_push':'Door_Push','shutdown':'END'})
    #Door_Init state
    smach.StateMachine.add('Door_Init',Door_Init(),transitions={'d_ready':'Door_Ready','cmd_modechg':'Mode_Chg','shutdown':'END'})
    #Door_Ready state
    smach.StateMachine.add('Door_Ready',Door_Ready(),transitions={'d_reach':'Door_Reach','cmd_modechg':'Mode_Chg','shutdown':'END'})
    #Door_Reach state
    smach.StateMachine.add('Door_Reach',Door_Reach(),transitions={'d_open':'Door_Open','cmd_modechg':'Mode_Chg','shutdown':'END'})
    #Door_Open state
    smach.StateMachine.add('Door_Open',Door_Open(),transitions={'d_push':'Door_Push','cmd_modechg':'Mode_Chg','shutdown':'END'})
    #Door_Push state
    smach.StateMachine.add('Door_Push',Door_Push(),transitions={'cmd_dinit':'Door_Init','cmd_modechg':'Mode_Chg','shutdown':'END'})

    #Egress_Mission State
    smach.StateMachine.add('Egress_Mission', Egress_Mission(), transitions={'e_init':'Egress_Init','cmd_modechg':'Mode_Chg','shutdown':'END'})
    smach.StateMachine.add('Egress_Init', Egress_Init(), transitions={'e_egress':'Egress_Egress','cmd_modechg':'Mode_Chg','shutdown':'END'})
    smach.StateMachine.add('Egress_Egress', Egress_Egress(), transitions={'e_standby':'Egress_Standby','cmd_modechg':'Mode_Chg','shutdown':'END'})
    smach.StateMachine.add('Egress_Standby', Egress_Standby(), transitions={'e_guide':'Egress_Guide','e_hello':'Egress_Hello','cmd_modechg':'Mode_Chg','shutdown':'END'})
    smach.StateMachine.add('Egress_Hello', Egress_Hello(), transitions={'e_guide':'Egress_Guide','cmd_modechg':'Mode_Chg','shutdown':'END'})
    smach.StateMachine.add('Egress_Guide', Egress_Guide(), transitions={'e_hello':'Egress_Hello','cmd_modechg':'Mode_Chg','shutdown':'END'})



    #Wall_Mission state
    smach.StateMachine.add('Wall_Mission',Wall_Mission(),transitions={'w_init':'Wall_Init','w_approach':'Wall_Approach','w_ready':'Wall_Ready','w_reach':'Wall_Reach','w_grab':'Wall_Grab','w_ungrab':'Wall_Ungrab','w_rotatedrill':'Wall_Rotatedrill','w_wallready':'Wall_Wallready','w_contact':'Wall_Contact','w_cut':'Wall_Cut','w_push':'Wall_Push','shutdown':'END'})
    #Wall_Init state
    smach.StateMachine.add('Wall_Init',Wall_Init(),transitions={'cmd_wready':'Wall_Ready','cmd_wapproach':'Wall_Approach','cmd_modechg':'Mode_Chg','shutdown':'END'})
    #Wall_Approach state
    smach.StateMachine.add('Wall_Approach',Wall_Approach(),transitions={'cmd_winit':'Wall_Init','cmd_modechg':'Mode_Chg','shutdown':'END'})
    #Wall_Ready state
    smach.StateMachine.add('Wall_Ready',Wall_Ready(),transitions={'cmd_wreach':'Wall_Reach','cmd_modechg':'Mode_Chg','shutdown':'END'})
    #Wall_Reach state
    smach.StateMachine.add('Wall_Reach',Wall_Reach(),transitions={'cmd_wgrab':'Wall_Grab','cmd_modechg':'Mode_Chg','shutdown':'END'})
    #Wall_Grab state
    smach.StateMachine.add('Wall_Grab',Wall_Grab(),transitions={'cmd_wungrab':'Wall_Ungrab','cmd_wroratedrill':'Wall_Rotatedrill', 'cmd_wwallready':'Wall_Wallready','cmd_modechg':'Mode_Chg','shutdown':'END'})
    #Wall_Ungrab state
    smach.StateMachine.add('Wall_Ungrab',Wall_Ungrab(),transitions={'cmd_wgrab':'Wall_Grab','cmd_wroratedrill':'Wall_Rotatedrill','cmd_modechg':'Mode_Chg','shutdown':'END'})
    #Wall_Rotatedrill state
    smach.StateMachine.add('Wall_Rotatedrill',Wall_Rotatedrill(),transitions={'cmd_wgrab':'Wall_Grab','cmd_wungrab':'Wall_Ungrab','cmd_modechg':'Mode_Chg','shutdown':'END'})
    #Wall_Wallready state
    smach.StateMachine.add('Wall_Wallready',Wall_Wallready(),transitions={'cmd_wcontact':'Wall_Contact','cmd_modechg':'Mode_Chg','shutdown':'END'})
    #Wall_Contact state
    smach.StateMachine.add('Wall_Contact',Wall_Contact(),transitions={'cmd_wcut':'Wall_Cut','cmd_modechg':'Mode_Chg','shutdown':'END'})
    #Wall_Cut state
    smach.StateMachine.add('Wall_Cut',Wall_Cut(),transitions={'cmd_wpush':'Wall_Push','cmd_modechg':'Mode_Chg','shutdown':'END'})
    #Wall_Push state
    smach.StateMachine.add('Wall_Push',Wall_Push(),transitions={'cmd_winit':'Wall_Init','cmd_modechg':'Mode_Chg','shutdown':'END'})

  # Execute SMACH plan
  sis = smach_ros.IntrospectionServer('dyros_jet', sm_top, '/SM_START')
  sis.start()
  outcome = sm_top.execute()
  rospy.spin()
  sis.stop()

  rospy.signal_shutdown('All done.')

if __name__ == '__main__':
  main()

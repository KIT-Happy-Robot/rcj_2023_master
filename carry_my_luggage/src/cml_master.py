#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Carry My Lugagge
# ルールブック：https://github.com/RoboCupAtHomeJP/Rule2022/blob/master/rules/opl/carry_my_luggage.md

import sys
import rospy
import roslib
import actionlib
import smach
import smach_ros

from std_msgs.msg import String, Float64
from happymimi_navigation.srv import NaviLocation
from enter_room.srv import EnterRoom
from happymimi_voice_msgs.srv import TTS, YesNo, ActionPlan
from actplan_executor.msg import APExecutorAction, APExecutorGoal
from find_bag.srv import FindBagSrv



tts_srv = rospy.ServiceProxy('/tts', TTS)


class FindBag(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['find_finish'])
        self.lr_sub = rospy.ServiceProxy("/left_right_recognition", String)
        self.fb_sub = rospy.ServiceProxy('/find_bag_server', FindBagSrv)

    def LRCB(self, msg):
        self.lrmsg = msg

    def execute(self, userdate):
        if self.lrmsg == 'right':
            self.fb_sub('right')

        return 'find_finish'


class Chaser(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=[''])


    def execute(self, userdate):
        
        return



class Return(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=[''])


    def execute(self, userdate):
        return




if __name__=='__main__':
    rospy.init_node('cml_master')
    rospy.loginfo("Start")
    sm_top = smach.StateMachine('finish_sm_top')

    with sm_top:
        smach.StateMachine.add(
            'FindBag',
            FindBag(),
            transitions={"":""})


        smach.StateMachine.add(
            'Chaser',
            Chaser(),
            transitions={"":""})

        smach.StateMachine.add(
            'Return',
            Return(),
            transitions={"":""})

    outcome = sm_top.execute()
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
from find_bag.srv import FindBagSrv, FindBagSrvResponse, GraspBagSrv, GraspBagSrvResponse



tts_srv = rospy.ServiceProxy('/tts', TTS)


class GraspBag(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes = ['grasp_finish',
                                        'grasp_retry'])

        self.lr_srv = rospy.ServiceProxy("/left_right_recognition", String)

        self.grasp  = rospy.ServiceProxy('/grasp_bag_server', GraspBagSrv)

    def LRCB(self, msg):
        self.lrmsg = msg

    def execute(self, userdate):
        answer = self.grasp().result
        if self.lrmsg == 'right':
            self.grasp('right')
            return 'grasp_finish'

        elif self.lrmsg == 'left':
            self.grasp('left')
            return 'grasp_finish'

        elif answer == False:
            rospy.sleep(0.5)
            return 'grasp_retry'


class Chaser(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['chaser_success'])



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
            'GraspBag',
            GraspBag(),
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
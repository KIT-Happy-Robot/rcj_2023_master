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
import time

from std_msgs.msg import String, Float64
from happymimi_navigation.srv import NaviLocation
#from enter_room.srv import EnterRoom
from happymimi_voice_msgs.srv import TTS, YesNo, ActionPlan
#from actplan_executor.msg import APExecutorAction, APExecutorGoal
#from find_bag.srv import FindBagSrv
from find_bag.srv import FindBagSrv, FindBagSrvResponse, GraspBagSrv, GraspBagSrvResponse
from base_control import BaseControl


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
        smach.State.__init__(self,outcomes=['chaser_finish'])

        self.chase = rospy.Publisher("/follow_human",String,queue_size=1)
        self.yesno = rospy.ServiceProxy('/yes_no', YesNo)
        self.start_time = time.time()

    def execute(self, userdate):
        tts_srv("/cml/follow_you")
        self.chase.publish('start')
        # while not rospy.is_shutdown():
        #     rospy.sleep(0.1)
        #     now_time = time.time() - self.start_time
        #     if self.find_msg == "lost" and now_time >= 5:
        #         tts_srv("I lost sight of you")
        #         tts_srv("Is this the location of the car?")
        #         answer = self.yesno().result
        #         if answer:
        
        return



class Return(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['return_finish'])

        self.navi = rospy.ServiceProxy("/navi_location_server",NaviLocation)
        self.base_control = BaseControl()

    def execute(self, userdate):
        rospy.loginfo('Executing state: RETURN')
        rospy.sleep(0.5)
        self.base_control.rotateAngle(170, 0.3)
        rospy.sleep(0.5)
        self.navi_srv('cml_start')
        rospy.sleep(0.5)
        tts_srv("/cml/finish_cml")
        return 'return_finish'





if __name__=='__main__':
    rospy.init_node('cml_master')
    rospy.loginfo("Start")
    sm_top = smach.StateMachine('finish_sm_top')

    with sm_top:
        smach.StateMachine.add(
            'GRASPBAG',
            GraspBag(),
            transitions = {"grasp_finish":"CHASER",
                            "grasp_retry":"GRASPBAG"})


        smach.StateMachine.add(
            'CHASER',
            Chaser(),
            transitions={"chaser_finish":"RETURN"})

        smach.StateMachine.add(
            'RETURN',
            Return(),
            transitions={"return_finish":"finish_sm_top"})

    outcome = sm_top.execute()
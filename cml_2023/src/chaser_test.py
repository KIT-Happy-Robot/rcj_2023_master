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
from sensor_msgs.msg import LaserScan
from happymimi_navigation.srv import NaviLocation
from happymimi_msgs.srv import StrTrg
from geometry_msgs.msg import Twist
from happymimi_voice_msgs.srv import TTS, YesNo, ActionPlan
from find_bag.srv import FindBagSrv, FindBagSrvResponse, GraspBagSrv, GraspBagSrvResponse
#from actplan_executor.msg import APExecutorAction, APExecutorGoal
#from find_bag.srv import FindBagSrv


base_path = roslib.packages.get_pkg_dir('happymimi_teleop') + '/src/'
sys.path.insert(0, base_path)
from base_control import BaseControl

tts_srv = rospy.ServiceProxy('/tts', TTS)
wave_srv = rospy.ServiceProxy('/waveplay_srv', StrTrg)

class Chaser(smach.State):      #timeup
    def __init__(self):
        smach.State.__init__(self,outcomes=['chaser_finish'])

        self.chase = rospy.Publisher("/follow_human",String,queue_size=1)

        rospy.Subscriber('/find_str', String, self.findCB)
        rospy.Subscriber('/cmd_vel', Twist, self.cmdCB)

        self.yesno = rospy.ServiceProxy('/yes_no', YesNo)
        self.arm = rospy.ServiceProxy('/servo/arm', StrTrg)

        self.start_time = time.time()
        self.find_msg = 'NULL'
        self.cmd_sub = 0.0
        self.cmd_count = 0

    def findCB(self, receive_msg):
        self.find_msg = receive_msg.data

    def cmdCB(self, receive_msg):
        self.cmd_sub = receive_msg.linear.x

    def execute(self, userdate):
        wave_srv("/cml/follow_you")
        self.chase.publish('start')
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            now_time = time.time() - self.start_time
            print(self.find_msg)
            #print(self.cmd_sub)
            #print("nt = ",now_time)
            ####
            if self.cmd_sub == 0.0 and self.find_msg == 'lost_stop':
                #self.find_msg = 'lost_stop'
                #self.start_time = time.time()
                #rospy.loginfo('loststoped')
                print("0.0 cmd = ",self.cmd_sub)
                print("0.0 nt = ",now_time)
                self.cmd_count += 1

                if now_time >= 5.0:
                    wave_srv("/cml/car_question")
                    rospy.loginfo('yes_or_no')
                    answer = self.yesno().result
                    if answer:
                        self.chase.publish('stop')
                        # self.base_control.rotateAngle(0, 0)
                        # self.base_control.translateDist(-0.3)
                        wave_srv('/cml/give_bag')
                        
                        self.arm('give')
                        wave_srv('/cml/return_start')
                        return 'chaser_finish'
                    else:
                        wave_srv("/cml/follow_cont")

            elif self.cmd_sub != 0.0:
                print("cmd = ",self.cmd_sub)
                print("nt = ",now_time)
                #self.find_msg = 'NULL'
                ###追加
                self.start_time = time.time()
                self.cmd_count = 0
                self.find_msg = 'lost_stop'
                ###
                
            elif self.cmd_count >= 30:
                return 'chaser_finish'

            else: 
                pass
        


if __name__=='__main__':
    rospy.init_node('cml_master')
    rospy.loginfo("Start")
    sm = smach.StateMachine(outcomes = ["finish_sm"])

    with sm:
        smach.StateMachine.add(
            'CHASER',
            Chaser(),
            transitions={"chaser_finish":"finish_sm"})


    outcome = sm.execute()

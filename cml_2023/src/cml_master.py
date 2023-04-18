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

from std_msgs.msg import String, Float64, Bool
from sensor_msgs.msg import LaserScan
from happymimi_navigation.srv import NaviLocation
from happymimi_msgs.srv import StrTrg
from geometry_msgs.msg import Twist
from happymimi_voice_msgs.srv import TTS, YesNo, ActionPlan
from find_bag.srv import FindBagSrv, FindBagSrvResponse, GraspBagSrv, GraspBagSrvResponse
#from actplan_executor.msg import APExecutorAction, APExecutorGoal
#from find_bag.srv import FindBagSrv\

base_path = roslib.packages.get_pkg_dir('happymimi_teleop') + '/src/'
sys.path.insert(0, base_path)
from base_control import BaseControl
# find_bag_path = roslib.packages.get_pkg_dir('find_bag') + '/src/'
# sys.path.insert(0, find_bag_path)
# from find_bag_server import FindBag

tts_srv = rospy.ServiceProxy('/tts', TTS)
wave_srv = rospy.ServiceProxy('/waveplay_srv', StrTrg)


class GraspBag(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes = ['grasp_finish',                                 
                                        'grasp_retry'])

        rospy.Subscriber("/left_right_recognition", String, self.LRCB)
        self.dist = rospy.Subscriber('/scan', LaserScan, self.laserCB)

        self.grasp = rospy.ServiceProxy('/grasp_bag_server', GraspBagSrv)
        
        self.arm_pose = rospy.ServiceProxy('/servo/arm', StrTrg)
        self.navi = rospy.ServiceProxy("/navi_location_server",NaviLocation)
        
        #self.eef_pub = rospy.Publisher('/servo/endeffector', Bool, queue_size=10)
        #rospy.Subscriber('/servo/endeffector', Bool)

        self.base_control = BaseControl()
        #self.FB = FindBag()
        self.lrmsg = "NULL"
        self.front_laser_dist = 0.0
        self.GB_count = 0
        self.right_count = 0
        self.left_count = 0

    def LRCB(self, msg):
        self.lrmsg = msg.data#!!

    def laserCB(self, receive_msg):
        self.front_laser_dist = receive_msg.ranges[359]

    def subscribeCheck(self):
        while not self.lrmsg and not rospy.is_shutdown():
            rospy.loginfo('No pose data available ...')
            rospy.sleep(1.5)

    def execute(self, userdate):
        #answer = self.grasp().result
        #tts_srv("which bag should I grasp")
    
        #rospy.sleep(3.0)
        wave_srv('/cml/start_cml')
        self.subscribeCheck()
        rospy.sleep(1.5)
        print(self.lrmsg)
        while not rospy.is_shutdown():
            print(self.lrmsg)
            rospy.sleep(0.6)
            if self.lrmsg == 'left':
                self.left_count += 1
                self.right_count = 0
                print("right_count = ",self.left_count)
                if self.left_count >= 5:
                    break

            elif self.lrmsg == 'right':
                self.right_count += 1
                self.left_count = 0
                print("left_count = ",self.right_count)
                if self.right_count >= 5:
                    break
            else:
                # self.left_count = 0
                # self.right_count = 0
                pass

        print("step2")
            
        while not rospy.is_shutdown():
            if self.right_count >= 5:
                wave_srv("/cml/bag_left")
                #tts_srv("right")
                rospy.loginfo('left')
                self.grasp('left', [0.25, 0.4])
                break

            elif self.left_count >= 5:
                wave_srv("/cml/bag_right")
                #tts_srv("left")
                rospy.loginfo('right')
                self.grasp('right', [0.25, 0.4])
                break

            else:
                pass

        rospy.sleep(3.0)

        if self.front_laser_dist > 0.2:
            return 'grasp_finish'

        elif self.front_laser_dist <= 0.2 and self.GB_count == 0:
            rospy.loginfo('Executing state: GRASP')
            rospy.sleep(0.5)
            ###追加
            self.base_control.translateDist(-0.3)
            self.base_control.rotateAngle(170, 0.3)
            rospy.sleep(0.5)
            self.navi('cml')
            rospy.sleep(0.5)
            ###
            
            # 下がって、バッグの向きへ微調整
            # self.base_control.translateDist(-0.3)
            # self.sleep(1.0)
            # dist_to_bag = self.FB.bagFocus('all', 100)
            # self.base_control.rotateAngle(4.0, 1, 0.7, 20)
            # rospy.sleep(0.5)
            # self.base_control.translateDist(dist_to_bag - 0.08 , 0.1)
            # rospy.sleep(0.5)
            # self.eef_pub.publish(True)
            # rospy.sleep(0.5)
            # self.arm_pose('carry')
            ###
            self.GB_count += 1
            return 'grasp_retry'

        # elif self.front_laser_dist <= 0.2 and self.GB_count == 0:   #rotateAngle 引数四つのほうがいいかも
        #     rospy.loginfo('Executing state: GRASP')
        #     rospy.sleep(0.5)
        #     print('retry')
        #     ###追加
        #     self.base_control.translateDist(-0.4)
        #     if self.left_count >= 5:    #右
        #         self.base_control.rotateAngle(20, 0, 0.5, 5)

        #     elif self.right_count >= 5: #左
        #         self.base_control.rotateAngle(-20, 0, 0.5, 5)
            
        #     self.base_control.translateDist(-0.9)
        #     ###
        #     self.GB_count += 1
        #     return 'grasp_retry'

        

        else:
            print("else")
            return 'grasp_finish'


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
            #print(self.cmd_sub)
            #print("nt = ",now_time)
            ####
            if self.cmd_sub == 0.0 and self.find_msg == 'lost_stop':
                #self.find_msg = 'lost_stop'
                #self.start_time = time.time()
                #rospy.loginfo('loststoped')
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
                print(self.cmd_sub)
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
            ####
            
            ####
        

class Return(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=["return_finish"])

        self.navi = rospy.ServiceProxy("/navi_location_server",NaviLocation)
        self.base_control = BaseControl()

    def execute(self, userdate):
        rospy.loginfo('Executing state: RETURN')
        rospy.sleep(0.5)
        self.base_control.rotateAngle(170, 0.3)
        rospy.sleep(0.5)
        self.navi('cml')
        rospy.sleep(0.5)
        wave_srv("/cml/finish_cml")
        return 'return_finish'



if __name__=='__main__':
    rospy.init_node('cml_master')
    rospy.loginfo("Start")
    sm = smach.StateMachine(outcomes = ["finish_sm"])

    with sm:
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
            transitions={"return_finish":"finish_sm"})

    outcome = sm.execute()

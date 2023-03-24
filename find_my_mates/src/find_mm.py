#!/usr/bin/env python3
#-*- coding:utf-8 -*-

# Find My Mates
# ルール（運営の議事録のコピー）：https://docs.google.com/document/d/1gJBUyupxfNYwTXRFuAvIxEVIj0Pn88jTKi9cPYdDmp4/edit?usp=sharing

# 
# 自律移動： hm_apps/hm_navigation/navi_location.py
# 人の座標位置推定：　human_coord_generator
# 人接近：hm_apps/approach_person　https://github.com/KIT-Happy-Robot/happymimi_apps/tree/develop/approach_person
# 人の検出： hm_recognition/recognition_processing/
# 
# 
# 
# 


import sys, os
import rospy
import std_msgs
import smach



from happymimi_navigation.srv import NaviLocation
 
 


file_path = roslib.packages.get_pkg_dir('happymimi_teleop') + '/src/'
sys.path.insert(0, file_path)
from base_control import BaseControl
# 足回り制御クラス
bc = BaseControl()
# 音声出力関数（サービスクライアント）
tts_srv = rospy.ServiceProxy('/tts', StrTrg)
wave_srv = rospy.ServiceProxy('/waveplay_srv', StrTrg)

# 例）左に９０度、0.5の角速度で回転する(右は角度マイナス)
#bc.rotateAngle(90, 0.5)

# 隣の部屋に移動・人接近・人の特徴取得
# 移動し、対象ゲストの方を向く→　ゲストの座標位置を取得する→　人接近する
# 自律移動： hm_apps/hm_navigation/navi_location.py
# 人の座標位置推定：　human_coord_generator
# 人接近：hm_apps/approach_person　https://github.com/KIT-Happy-Robot/happymimi_apps/tree/develop/approach_person
# YesOrNo、
# 特徴取得系： hm_recognition/person_feature_extraction/src
# 　服の色検出：　https://github.com/KIT-Happy-Robot/happymimi_recognition/blob/master/person_feature_extraction/src/detect_cloth_color.py
# 　眼鏡のありなし：　https://github.com/KIT-Happy-Robot/happymimi_recognition/blob/master/person_feature_extraction/src/detect_glass.py
# 　髪の色：　https://github.com/KIT-Happy-Robot/happymimi_recognition/blob/master/person_feature_extraction/src/detect_hair_color.py
# 
class GetFeature(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['approach_finish'],
                             input_keys = ['g_num_in','feature_in'],
                             output_keys = ['g_num_out','feature_out'])

        self.coord_gen_srv = rospy.ServiceProxy('/human_coord_generator',SimpleTrg)
        self.ap_srv = rospy.ServiceProxy('/approach_person_server', StrTrg)
        self.navi_srv = rospy.ServiceProxy('navi_location_server', NaviLocation)
        self.glass_srv = rospy.ServiceProxy('/person_feature/glass', StrToStr)
        self.height_srv = rospy.ServiceProxy('/person_feature/height',SetFloat)
        self.cloth_color_srv = rospy.ServiceProxy('	/person_feature/cloth_color',SetStr)
        
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)


    def execute(self, userdata):
        self.features = []
        self.features = userdata.features

        g_num = userdata.g_num_in
        g_name = "human_" + str(g_num)

        # 隣の部屋（Living_room）まで移動 
        # tts_srv("Move to guest")
        wave_srv("/fmm/move_guest")
        rospy.sleep(0.5)
        self.navi_srv('living room')

        # g_numが0だったら、一人目の方を向いて座標を取得する→　接近→　名前を確認する→　特徴を取得
        # 　名前の確認では、音声会話から名前の特定をする
        if g_num == 0:
            #0(水平)１(下に1°)-1(上に1°)
            self.head_pub.publish(0)
            rospy.sleep(1.0)


        
        # g_numが1だったら、2人目の方を～～
        
        # g_numが2だったら、3人目の方を～～



        # 名前(g_1とかg_2)と特徴を一緒のYamlファイルに保存
        f = "g_" + str(userdata.g_num_in) + ".yaml"
        with open(os.path.join(roslib.packages.get_pkg_dir("find_mm") + "/src")) as g_file:
            yaml.dump()

        # 

        return 'approach_finish'



# class State(smach.State):


def smach():
    sm = smach.StateMachine(outcomes = ['fmm_finish'])
    sm.userdata.g_num = 0 # 完了したゲストの数

    with sm:
        smach.StateMachine.add("GetFeature",
                               GetFeature(),
                               transitions = {"approach_finish":"Tell",
                                              "":"" },
                               remapping = {"g_num_in":"g_num",
                                            "g_num_out":"g_num"})
        smach.StateMachine.add("Tell",
                               Tell(),
                               transitions = {"":""},
                               remapping = {"":""})


if __name__=='__main__':

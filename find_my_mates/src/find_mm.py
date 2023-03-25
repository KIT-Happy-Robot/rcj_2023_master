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

from happymimi_msgs.srv import StrToStr, SetTrg, SetFloat, SimpleTrg# , SetStr
from happymimi_navigation.srv import NaviLocation
from happymimi_voice_msgs.srv import TTS, YesNo, StringToString
from 


import roslib
file_path = roslib.packages.get_pkg_dir('happymimi_teleop') + '/src/'
sys.path.insert(0, file_path)
from base_control import BaseControl
# 足回り制御クラス
bc = BaseControl()
# 音声出力関数（サービスクライアント）
tts_srv = rospy.ServiceProxy('/tts', StrTrg)
wave_srv = rospy.ServiceProxy('/waveplay_srv', StrTrg)


# 人の目の前までに寄る状態
# 隣の部屋に移動・人接近・人の特徴取得
# 移動し、対象ゲストの方を向く→　ゲストの座標位置を取得する→　人接近する
# 自律移動： hm_apps/hm_navigation/navi_location.py
# 人の座標位置推定：　human_coord_generator
# 人接近：hm_apps/approach_person　https://github.com/KIT-Happy-Robot/happymimi_apps/tree/develop/approach_person
class GetClose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['get_close_finish'],
                            input_keys = ['g_num_in'],
                            output_keys = ['g_num_out'])
        self.coord_gen_srv = rospy.ServiceProxy('/human_coord_generator',SimpleTrg)
        self.ap_srv = rospy.ServiceProxy('/approach_person_server', StrTrg)
        self.navi_srv = rospy.ServiceProxy('navi_location_server', NaviLocation)

    def execute():
        g_num = userdata.g_num_in
        g_name = "human_" + str(g_num)

        # 隣の部屋（Living_room）まで移動 
        wave_srv("/fmm/move_guest")  # tts_srv("Move to guest")に等しい
        rospy.sleep(0.5)
        self.navi_srv('living room')

        # g_numが0だったら、一人目の方を向いて座標を取得する→　接近→　名前を確認する→　特徴を取得
        # 　名前の確認では、音声会話から名前の特定をする
        if g_num == 0:
            #0(水平)１(下に1°)-1(上に1°)
            self.head_pub.publish(0)
            rospy.sleep(1.0)
            bc.rotateAngle(-5, 0.5)
            self.coord_gen_srv()
            self.ap_srv(data = g_name)
            self.head_pub.publish(-20)
            rospy.sleep(1.0)



# 例）左に９０度、0.5の角速度で回転する(右は角度マイナス)
#bc.rotateAngle(90, 0.5)

# YesOrNo、

class GetFeature(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['get_feature_finish'],
                             input_keys = ['g_num_in','feature_in'],
                             output_keys = ['g_num_out','feature_out'])

        # Features
        # https://github.com/KIT-Happy-Robot/happymimi_voice/blob/master/happymimi_voice_common/src/get_feature_srv.py
        self.gf_srv= rospy.ServiceProxy('get_feature_srv', StrToStr)
        self.glass_srv = rospy.ServiceProxy('/person_feature/glass', StrToStr)
        self.height_srv = rospy.ServiceProxy('/person_feature/height',SetFloat)
        self.cloth_color_srv = rospy.ServiceProxy('/person_feature/cloth_color',SetStr)
        self.getold_srv = rospy.ServiceProxy('/person_feature/old', SetStr)
        self.getgender_srv = rospy.Service('/gender_jg', StringToString)
        self.height_srv = rospy.ServiceProxy('/person_feature/height_estimation', SetFloat)
        self.cloth_srv  = rospy.ServiceProxy('/person_feature/cloth_color', SetStr)
        
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)

        self.guest_name  = "null"
        self.ffv = FeatureFromVoice()


    # 「～さんですか？」って聞いてって名前を特定する関数
    # 画像で名前を判断したいな https://www.panasonic.com/jp/business/its/ocr/ai-ocr.html
    def getName(self):
        # tts_srv("Excuse me. I have a question for you")
        wave_srv("/fmm/start_q")
        

        for i in range(3):
            name_res = self.gf_srv(req_data = name) # voiceのgetName()の結果(string res_data, bool result)を格納
            self.guest_name = self.ffv.getName()
        
            # getNameがTrueなら
            if name_res.result:
                self.guest_name = name_res.res_data # 返答された名前を格納
                break
            # 
            elif i < 3:
                pass
            # Falseかつ3回目まできたとき
            else:
                self.guest_name = "" # 空の名前
        tts_srv("Hi!" + self.guest_name)
        return self.guest_name


    # 画像認識の特徴取得系： hm_recognition/person_feature_extraction/src
    # 　服の色検出：　https://github.com/KIT-Happy-Robot/happymimi_recognition/blob/master/person_feature_extraction/src/detect_cloth_color.py
    # 　眼鏡のありなし：　https://github.com/KIT-Happy-Robot/happymimi_recognition/blob/master/person_feature_extraction/src/detect_glass.py
    # 　髪の色：　https://github.com/KIT-Happy-Robot/happymimi_recognition/blob/master/person_feature_extraction/src/detect_hair_color.py
    # 
    # 使用済みの特徴を飛ばすシステムを作りたいよね
    def getAge(self):
        self.old_year = 0

        self.old_year = int(self.getold_srv().result)

        if self.old_year < 20: return " looks under 20"
        elif self.old_year >= 20 and self.old_year < 30: return " looks in the twenties"
        elif self.old_year >= 30 and self.old_year < 40: return " looks in the 30s"
        elif self.old_year >= 40 and self.old_year < 50: return " looks in the 40s"
        elif self.old_year >= 50 and self.old_year < 60: return " looks in the 50s"
        elif self.old_year >= 60 and self.old_year < 70: return " looks in the 60s"
        else:
            return "so old!!" 

    # https://github.com/KIT-Happy-Robot/rcj_2022_master/blob/develop/find_my_mates2022/src/fmmmod.py
    def getGender(self, msg):
        self.sex = "null"
        res = self.getgender_srv(msg)
        if res.result:
            self.sex=res.result_data 
        else:
            self.sex = "null"
        tts_srv("You are " + self.sex)
        rospy.loginfo(self.sex)
        return self.sex
        
    def getHight(self):
        self.head_pub.publish(0)
        # 全身を収めるために後ろへ下がる
        self.base_control.translateDist(-0.5,0.2)
        
        height = SetFloat()
        height = self.height_srv()
        
        if height.data == -1:
            return False
        else:
            self.height = str(round(height.data))
            return self.height
            
    def getClothColor(self):
        self.cloth_color = "null"
        self.cloth_color = self.cloth_srv().result
        if self.cloth_color == '':
            return "none"
        else:
            return self.cloth_color
            
    def getHairColor(self):
        self.hair_color = "null"
        self.hair_color = self.hair_srv().result
        if self.hair_color == '':
            return "none"
        else:
            return self.hair_color
            
    def skinColor(self):
        self.skin_color = "null"
        self.skin_color =  self.skin_srv().result
        if self.skin_color == '':
            return "none"
        else:
            return self.skin_color
            
    def getGlass(self):
    def getLocInfo(self):


    def execute(self, userdata):
        self.features = []
        self.features = userdata.features

        # 使用済みの特徴を使わないようにする

        g_num = userdata.g_num_in
        g_name = "human_" + str(g_num)

        
        # g_numが1だったら、2人目の方を～～
        
        # g_numが2だったら、3人目の方を～～

        # 各ゲストの特徴を保存

        # 

        return 'approach_finish'

# ゲスト度に取得した特徴２つをオペレーターへ伝える状態
class Tell(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes = ['tell_finish'],
                             input_keys = ['g_num_in','features_in'])
                             #output_keys = ['features_out'])
        self.navi_srv =  rospy.ServiceProxy('navi_location_server', NaviLocation)
        self.save_srv = rospy.ServiceProxy('/recognition/save', StrTrg)
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size=1)
        self.bc = BaseControl()
        self.sentence_list = []
        self.si = SaveInfo()

    def execute(self, userdata):
        # inputとoutptに気を付ける
        count_num = userdata.g_num_in
        self.sentence_list = userdata.feature_in
        
        
        # 首の角度を０度に戻す
        self.head_pub.publish(0)
        rospy.sleep(1.0)
        # オペレーターへ自律移動
        self.navi_srv('operator')
        #　首を上げる
        self.head_pub.publish(-20)

        
        # 取得した名前とそれに紐づけた特徴２つを音声で出力する
        #
        # 

        return "tell_finish"


def smach():
    sm = smach.StateMachine(outcomes = ['fmm_finish'])
    sm.userdata.g_num = 0 # 完了したゲストの数
    sm.userdata.features = []

    with sm:
        smach.StateMachine.add("GetClose",
                               GetClose(),
                               transitions = {"get_close_finish":"GetFeature"},
                               remapping = {"g_num_in":"g_num",
                                            "g_num_out":"g_num"})
        smach.StateMachine.add("GetFeature",
                               GetFeature(),
                               transitions = {"get_feature_finish":"Tell"},
                               remapping = {"g_num_in":"g_num",
                                            "g_num_out":"g_num",
                                            "features_in":"features"})
        smach.StateMachine.add("Tell",
                               Tell(),
                               transitions = {"tell_finish":"GetClonse"},
                               remapping = {"feature_in":""})


if __name__=='__main__':

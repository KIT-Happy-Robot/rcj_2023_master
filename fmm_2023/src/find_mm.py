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
import yaml
import rosparam
import roslib.packages
from scipy.spatial import distance
from happymimi_msgs.srv import StrToStr, StrTrg, SetFloat, SimpleTrg, SetStr
from happymimi_navigation.srv import NaviLocation
from std_msgs.msg import Float64
from happymimi_voice_msgs.srv import TTS, YesNo, StringToString
# from fmmmod import FeatureFromVoice, FeatureFromRecog,  LocInfo, SaveInfo
from happymimi_navigation.srv import NaviLocation, NaviCoord
from happymimi_voice_msgs.srv import StringToString,StringToStringResponse
#from std_srvs.srv import Empty
from happymimi_voice_msgs.srv import SpeechToText
from happymimi_msgs.srv import StrToStrResponse
# import re
# import fuzzy
# import copy
# import math
happymimi_voice_path=roslib.packages.get_pkg_dir("happymimi_voice")+"/.."
sys.path.insert(0,happymimi_voice_path)
from happymimi_nlp import sentence_analysis as se
from happymimi_nlp import gender_judgement_from_name as GetGender
import pickle 

file_path = roslib.packages.get_pkg_dir('happymimi_teleop') + '/src/'
sys.path.insert(0, file_path)
from base_control import BaseControl
# 足回り制御クラス

# 音声出力関数（サービスクライアント）
tts_srv = rospy.ServiceProxy('/tts', StrTrg)
wave_srv = rospy.ServiceProxy('/waveplay_srv', StrTrg)

file_path=happymimi_voice_path+"/config/voice_common"
file_temp="/get_feature.txt"
name_path=roslib.packages.get_pkg_dir("fmm_2023")+"/config/guest_name.yaml"

pkl_name_path=roslib.packages.get_pkg_dir("fmm_2023")+"/config/guest_name.pkl"
happymimi_voice_path=roslib.packages.get_pkg_dir("happymimi_voice")+"/.."
sys.path.insert(0,happymimi_voice_path)

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
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        self.bc = BaseControl()

    def execute(self, userdata):
        rospy.loginfo("Executing state: APPROACH_GUEST")
        g_num = userdata.g_num_in
        print(g_num)
        print(type(g_num))
        g_name = "human_" + str(g_num)
        if g_num == 0:
            # tts_srv("Start Find My Mates")
            wave_srv("/fmm/start_fmm")
        self.bc.rotateAngle(180, 0, 1.0, 10)
        # 隣の部屋（Living_room）まで移動 
        wave_srv("/fmm/move_guest")  # tts_srv("Move to guest")に等しい
        rospy.sleep(0.5)
        self.navi_srv('living')

        # g_numが0だったら、一人目の方を向いて座標を取得する→　接近→　名前を確認する→　特徴を取得
        # 　名前の確認では、音声会話から名前の特定をする
        if g_num == 0:
            #0(水平)１(下に1°)-1(上に1°)
            self.head_pub.publish(0)
            rospy.sleep(1.0)
            rospy.set_param("/map_range/min_x", -2.5)
            rospy.set_param("/map_range/max_x", 1.0)
            rospy.set_param("/map_range/min_y", 3.0)
            rospy.set_param("/map_range/max_y", 5.3)
            result = self.coord_gen_srv().result
            print(result)
            if result == False:
                while True:
                    if result == False:
                        result = self.coord_gen_srv().result
                        print(result)
            else:
                pass
            try:
                result = self.ap_srv(data = "human_0").result #g_name
                print(result)
            except KeyError:
                while True:
                    if result == False:
                        result = self.ap_srv(data = "human_0").result #g_name
                        print(result)
                    else:
                        break

        
        elif g_num == 1:
            self.head_pub.publish(0)
            rospy.sleep(1.0)
            #self.bc.translateDist(1.0,0.2)
            #rospy.sleep(1.0)
            #self.bc.rotateAngle(-90,1.0)
            #rospy.sleep(1.0)
            
          
            self.bc.rotateAngle(-50, 0, 0.5, 5)
            rospy.sleep(1.0)
            self.bc.translateDist(0.5,0.2)
            
            rospy.set_param("/map_range/min_x", -0.3)
            rospy.set_param("/map_range/max_x", 1.4)
            rospy.set_param("/map_range/min_y", 2.8)
            rospy.set_param("/map_range/max_y", 4.6)
            result = self.coord_gen_srv().result
            print(result)
            if result == False:
                while True:
                    if result == False:
                        result = self.coord_gen_srv().result
                        print(result)
            #self.ap_srv(data = "human_0")
            try:
                result = self.ap_srv(data = "human_0").result #g_nameif result == False:
                while True:
                    if result == False:
                        result = self.coord_gen_srv().result
                        print(result)
            #self.ap_srv(data = "human_0") #g_name
                print(result)
            except KeyError:
                while True:
                    if result == False:
                        result = self.ap_srv(data = "human_0").result #g_name
                        print(result)
                    else:
                        break
                
        elif g_num == 2:
            self.head_pub.publish(0)
            rospy.sleep(1.0)
            #self.bc.translateDist(1.0,0.2)
            #rospy.sleep(1.0)
            #self.bc.rotateAngle(-90,1.0)
            #rospy.sleep(1.0)
            #self.bc.translateDist(0.5,0.2)
            self.bc.rotateAngle(-100, 0, 0.5, 5)
            rospy.set_param("/map_range/min_x", -0.8)
            rospy.set_param("/map_range/max_x", 1.4)
            rospy.set_param("/map_range/min_y", 1.4)
            rospy.set_param("/map_range/max_y", 2.6)
            
            result = self.coord_gen_srv().result
            print(result)
            if result == False:
                while True:
                    if result == False:
                        result = self.coord_gen_srv().result
                        print(result)
            #self.ap_srv(data = "human_0") #g_name
            try:
                result = self.ap_srv(data = "human_0").result #g_name
                print(result)
            except KeyError:
                while True:
                    if result == False:
                        result = self.ap_srv(data = "human_0").result #g_name
                        print(result)
                    else:
                        break

        else:
            pass
        #result = self.ap_srv(data = guest_name)
        #print(result)
        self.head_pub.publish(0)
        if result:
            return 'get_close_finish'
        else:
            # 失敗のパターン
            #return 'get_close_false'
            return 'get_close_finish'


# 例）左に９０度、0.5の角速度で回転する(右は角度マイナス)
#bc.rotateAngle(90, 0.5)

# YesOrNo、

class GetFeature(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['get_feature_finish'],
                             input_keys = ['g_num_in'],
                             output_keys = ['feature_out'])
        
        #(self, outcomes = ['get_feature_finish'],
        #                     input_keys = ['g_num_in','feature_in'],
        #                     output_keys = ['g_num_out','feature_out'])


        # Features
        # https://github.com/KIT-Happy-Robot/happymimi_voice/blob/master/happymimi_voice_common/src/get_feature_srv.py
        self.gf_srv= rospy.ServiceProxy('get_feature_srv', StrToStr)
        self.glass_srv = rospy.ServiceProxy('/person_feature/glass', StrToStr)
        #self.height_srv = rospy.ServiceProxy('/person_feature/height',SetFloat)
        self.getold_srv = rospy.ServiceProxy('/person_feature/old', SetStr)
        self.getgender_srv = rospy.ServiceProxy('/person_feature/gender', SetStr)
        self.height_srv = rospy.ServiceProxy('/person_feature/height_estimation', SetFloat)
        self.cloth_srv  = rospy.ServiceProxy('/person_feature/cloth_color', SetStr)
        self.hair_color_srv = rospy.ServiceProxy('/person_feature/hair_color', SetStr)
        self.feature_srv = rospy.ServiceProxy('get_feature_srv', StrToStr)
        self.yes_no_srv = rospy.ServiceProxy('/yes_no', YesNo)
        
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)

        self.guest_name  = "null"
        self.guest_loc   = "null"
        self.gn_sentence = "null"
        self.f1_sentence = "null"
        self.f2_sentence = "null"
        self.sentence_list = []

        self.loc_dict   = rospy.get_param('/location')
        self.human_dict = {}
        self.loc_name_list = list(self.loc_dict.keys())
        self.loc_name      = "null"
        
        
        self.result = 0.00
        self.loc_result = "null"        
        self.bc = BaseControl()

    # 「～さんですか？」って聞いてって名前を特定する関数
    # 画像で名前を判断したいな https://www.panasonic.com/jp/business/its/ocr/ai-ocr.html
 #   def getName(self):
 #       with open(pkl_name_path,"rb") as pf:
  #          names = pickle.load(pf)
   #     if names:
    #        ans_name = ""
     #       for name in names:
      #          if name == names[-1]:
       #             ans_name = names[-1]
        #            break
         #       else:
          #          tts_srv("Are you" + name)
           #         yes_no = self.yes_no_srv().result
            #        if yes_no:
             #           ans_name = name
              #          break
               #     else:
                #        continue
#            names.remove(ans_name)
 #           with open(pkl_name_path,"wb") as pf:
  #              pickle.dump(names,pf)
   #     else:
    #        ans_name = None

     #   return ans_name
    def getName(self):
        self.name = "null"
        for i in range(3):
            name_res = self.feature_srv(req_data = "fmm name")
            print (name_res.res_data)
            if name_res.result:
                self.name = name_res.res_data
                tts_srv("Hi " + self.name)
                break
            elif i == 3:
                break
                # tts_srv("Sorry. I'm going to ask you one more time.")
            else:
                wave_srv("/fmm/ask_again")
                self.name = "guest"
        return self.name


    # 画像認識の特徴取得系： hm_recognition/person_feature_extraction/src
    # 　服の色検出：　https://github.com/KIT-Happy-Robot/happymimi_recognition/blob/master/person_feature_extraction/src/detect_cloth_color.py
    # 　眼鏡のありなし：　https://github.com/KIT-Happy-Robot/happymimi_recognition/blob/master/person_feature_extraction/src/detect_glass.py
    # 　髪の色：　https://github.com/KIT-Happy-Robot/happymimi_recognition/blob/master/person_feature_extraction/src/detect_hair_color.py
    # 
    # 使用済みの特徴を飛ばすシステムを作りたいよね
    def getAge(self):
        self.old_year = 0

        #self.old_year = int(self.getold_srv().result)
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
    def getGender(self, msg=[""]):
        self.sex = "null"
        res = self.getgender_srv()
        if res.result == "":
            #self.sex=res.result_data 
            self.sex = "null"
        else:
            self.sex = res.result
        tts_srv("You are " + self.sex)
        rospy.loginfo(self.sex)
        return self.sex
        
    def getHight(self):
        self.head_pub.publish(0)
        # 全身を収めるために後ろへ下がる
        self.bc.translateDist(-1.5,0.2)
        self.head_pub.publish(15)
        
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
        self.hair_color = self.hair_color_srv().result
        if self.hair_color == '':
            return "none"
        else:
            return self.hair_color
            
    def getSkinColor(self):
        self.skin_color = "null"
        self.skin_color =  self.skin_srv().result
        if self.skin_color == '':
            return "none"
        else:
            return self.skin_color
            
    def getGlass(self):# わからんから適当
        #self.glass = "null"
        self.glass_result = self.glass_srv().result # T/F
        self.glass_data = self.glass_srv().res_data
        print(self.glass_result)
        #if self.glass_result:
        if self.glass_data == "Normal":
            return 'wearing'
        else:
            return "no wearing"

    def getLocInfo(self, target_name):
        self.loc_name = "null"
        self.human_dict = rospy.get_param('/tmp_human_location')
        print(self.human_dict)
        h_rpy = self.human_dict[target_name]
        h_xy = (h_rpy[0], h_rpy[1])
        for i in range(len(self.loc_name_list)):
            self.loc_name = self.loc_name_list[i]
            loc_rpy = self.loc_dict[self.loc_name]
            l_xy = (loc_rpy[0], loc_rpy[1])
            if i == 0:
                stdval = distance.euclidean(h_xy, l_xy)
            dist = distance.euclidean(h_xy, l_xy)
            #print (self.loc_name)
            #print (dist)
            if stdval > dist:
                stdval = dist
                self.loc_result = self.loc_name
        print (self.loc_result)
        return self.loc_result
        
    def execute(self, userdata):
        #self.features = []
        #self.features = userdata.feature_in
        rospy.loginfo("Executing state: FIND_FUATURE")
        self.head_pub.publish(-20)
        #g_name = "human_" + str(g_num)
        #tts_srv("Excuse me. I have a question for you")
        #wave_srv("/fmm/start_q")
        self.guest_name = self.getName()
        g_num = userdata.g_num_in
        #print (self.guest_name)
        #self.guest_loc = self.getLocInfo("human_" + str(g_num))
        self.guest_loc = self.getLocInfo("human_0")
        self.gn_sentence = str(self.guest_name) + " is near " + str(self.guest_loc)
        # 使用済みの特徴を使わないようにする

        if g_num == 0:
            self.bc.translateDist(-0.4, 0.2)

            #self.f1_sentence = "ClothColor is " + self.getClothColor()
            print('startglass') 
            
            self.f1_sentence = "Glass is " + self.getGlass()
            print(self.f1_sentence)
            #self.f2_sentence = self.getGlass() + "glass"
            self.f2_sentence = "Gender is " + self.getGender()
            print(self.f2_sentence)
            
        # g_numが1だったら、2人目の方を～～
        elif g_num == 1:
            self.bc.translateDist(-0.4, 0.2)

            self.f1_sentence = "Age is " + self.getAge()
            self.f2_sentence = "ClothColor is " + self.getClothColor()
            
        # g_numが2だったら、3人目の方を～～
        elif g_num == 2:
            self.bc.translateDist(-0.4, 0.2)

            self.f1_sentence = "HairColor is " + self.getHairColor()
            #glassのリターン変えたほうがいいかも
            #self.f2_sentence = "Age is " + self.getAge()
            self.f2_sentence = "Hight is" + self.getHight()
        else:
            return 'get_feature_finish'
        # 各ゲストの特徴を保存
        # tts_srv("Thank you for your cooperation")
        wave_srv("/fmm/finish_q")
        print(self.f1_sentence)
        print(self.f2_sentence)
        #userdata.feature_out = [self.gn_sentence, self.f1_sentence, self.f2_sentence]
        feature_list = [self.gn_sentence, self.f1_sentence, self.f2_sentence]
        userdata.feature_out = feature_list
        return 'get_feature_finish'

# ゲスト度に取得した特徴２つをオペレーターへ伝える状態
class Tell(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes = ['tell_finish','all_finish'],
                             input_keys = ['g_num_in','feature_in'],
                             output_keys = ['g_num_out'])
        self.navi_srv = rospy.ServiceProxy('navi_location_server', NaviLocation)
        self.save_srv = rospy.ServiceProxy('/recognition/save', StrTrg)
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size=1)
        self.bc = BaseControl()
        self.sentence_list = []
        self.data_path = roslib.packages.get_pkg_dir("fmm_2023") + "/guest_info/"
        
    def saveInfo(self, name, data):
        rospy.loginfo('Save feature')
        file_name = name + ".yaml"
        with open(os.path.join(self.data_path, file_name), "w") as yf:
            yaml.dump(data, yf, default_flow_style = False)
        # self.save_srv(data = self.data_path)

    def execute(self, userdata):
        # inputとoutptに気を付ける
        count_num = userdata.g_num_in
        self.sentence_list = userdata.feature_in
        print(userdata.feature_in)
        print(userdata.g_num_in)
        print(self.sentence_list)
        wave_srv("/fmm/move_operator")
        
        # 首の角度を０度に戻す
        # self.head_pub.publish(0)
        # rospy.sleep(0.5)
        
        # オペレーターへ自律移動
        self.bc.rotateAngle(180, 1, 0.7, 20)
        rospy.sleep(0.5)
        #self.navi_srv('operator')
        navi_result = self.navi_srv('fmm').result
        rospy.sleep(0.2)
        #　首を上げる
        self.head_pub.publish(-20)
        rospy.sleep(0.2)
        
        # 取得した名前とそれに紐づけた特徴２つを音声で出力する
        if navi_result:
            # tts_srv("I'll give you the guest information.")
            wave_srv('/fmm/start_req')
        else:
            # tts_srv("I'm sorry. I couldn't navigate to the operator's location. I will provide the features from here.")
            wave_srv("/fmm/start_req_here")
        #print(self.sentence_list)
        for i in range(len(self.sentence_list)):
            tts_srv(self.sentence_list[i])
            i += 1

        self.saveInfo("guest_" + str(count_num), self.sentence_list)
        userdata.g_num_out = count_num + 1
        
        if count_num >= 2: 
            # tts_srv("Finish Find My Mates. Thank you very much")
            wave_srv("/fmm/finish_fmm")
            return 'all_finish'
        else:
            return 'tell_finish'


#def smach():
    # sm = smach.StateMachine(outcomes = ['fmm_finish'])
    # sm.userdata.g_num = 0 # 完了したゲストの数
    # sm.userdata.features = []

    # with sm:
    #     smach.StateMachine.add("GetClose",
    #                            GetClose(),
    #                            transitions = {"get_close_finish":"GetFeature"},
    #                            remapping = {"g_num_in":"g_num",
    #                                         "g_num_out":"g_num"})
    #     smach.StateMachine.add("GetFeature",
    #                            GetFeature(),
    #                            transitions = {"get_feature_finish":"Tell"},
    #                            remapping = {"g_num_in":"g_num",
    #                                         "g_num_out":"g_num",
    #                                         "features_in":"features"})
    #     smach.StateMachine.add("Tell",
    #                            Tell(),
    #                            transitions = {"tell_finish":"GetClonse"},
    #                            remapping = {"feature_in":"features"})
    # outcome = sm.execute()

if __name__=='__main__':
    rospy.init_node('find_mm')
    rospy.loginfo("Start Find My Mates")

    # sm = smach()
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
                                            "features_in":"features",
                                            "feature_out":"features"})
        smach.StateMachine.add("Tell",
                               Tell(),
                               transitions = {"tell_finish":"GetClose",
                                              "all_finish":"fmm_finish"},
                               remapping = {"feature_in":"features",
                                            "g_num_in":"g_num",
                                            "g_num_out":"g_num"})
    outcome = sm.execute()
#!/usr/bin/env python3
from __future__ import print_function

import os
import sys
import rospy
import cv2
import csv
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from skimage.transform import resize
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelStates
import numpy as np
import roslib
import copy
sys.path.append(os.path.join(os.path.dirname(__file__), 'pytorch'))
from nav_cloning_pytorch import *

class nav_cloning_node:
    def __init__(self):
        rospy.init_node('nav_cloning_node', anonymous=True)
        self.action_num = 1
        self.dl = deep_learning(n_action=self.action_num)  # deep_learningクラスを初期化
        self.bridge = CvBridge()
        
        self.robot_name = "turtlebot3"  # 例（実際のモデル名に合わせて）
        self.image_sub = rospy.Subscriber("/camera/lane1/center/rgb/image_raw", Image, self.callback)
        self.vel_sub = rospy.Subscriber("/nav_vel", Twist, self.callback_vel)
        self.gazebo_pos_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback_gazebo_pos, queue_size = 2)
        self.waypoint_num = rospy.Subscriber("/count_waypoint", Int8, self.callback_waypoint)

        self.action_pub = rospy.Publisher("action", Int8, queue_size=1)
        self.nav_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.cv_image = np.zeros((520, 694, 3), np.uint8)
        self.vel = Twist()
        self.pro = "20250517_12:49:45"  # モデルファイル
        self.model_num = rospy.get_param("/nav_cloning_node/model_num", "1")
        self.load_path = roslib.packages.get_pkg_dir('nav_cloning') + f'/data/model/{self.pro}/model{self.model_num}.pt'
        self.score = roslib.packages.get_pkg_dir('nav_cloning') + f'/data/score/{self.pro}.csv'
        self.dl.load(self.load_path)
        print(self.load_path)
        
        self.collision_list = [[], []]
        self.gazebo_pos_x = 0.0
        self.gazebo_pos_y = 0.0
        
        self.loop_check_interval = 5  # 何ステップごとに周回判定をするか（過剰検出防止）
        self.step_counter = 0
        self.current_wp = 0
        
        self.is_finished = False
        self.is_success = False
        
    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def callback_vel(self, data):
        self.vel = data

    def callback_gazebo_pos(self, data):
        if self.robot_name in data.name:
            i = data.name.index(self.robot_name)
            self.gazebo_pos_x = data.pose[i].position.x
            self.gazebo_pos_y = data.pose[i].position.y
    
    def callback_waypoint(self, data):
        self.current_wp = data.data
        
    def collision(self):
        self.collision_list[0].append(self.gazebo_pos_x)
        self.collision_list[1].append(self.gazebo_pos_y)

        if len(self.collision_list[0]) == 10:
            avg_x = np.mean(self.collision_list[0])
            avg_y = np.mean(self.collision_list[1])
            distance = np.sqrt((self.gazebo_pos_x - avg_x)**2 + (self.gazebo_pos_y - avg_y)**2)

            self.collision_list[0].pop(0)
            self.collision_list[1].pop(0)
            
            if distance < 0.1:
                rospy.logwarn('model_' + str(self.model_num) + 'Collision detected! Quit the program')
                self.is_finished = True
                self.is_success = False
            
        return
    
    def write_score(self):
        # 書き込みたい内容
        self.line = ['model_' + str(self.model_num), str(self.is_success), str(self.step_counter), str(self.gazebo_pos_x), str(self.gazebo_pos_y)]

        # CSV全体を読み込む（存在しない場合は空のリスト）
        if os.path.exists(self.score):
            with open(self.score, 'r') as f:
                lines = list(csv.reader(f))
        else:
            lines = []

        # 必要なら空行を追加しておく
        while len(lines) <= int(self.model_num):
            lines.append([''] * len(self.line))  # 空の行（列数は合わせる）

        # 対象の行を更新
        lines[int(self.model_num)] = self.line

        # 全体を上書き
        with open(self.score, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(lines)

    def loop(self):
        # 画像サイズが正しいか確認
        if self.cv_image.shape != (480, 640, 3):
            return

        #ステップ更新
        self.step_counter += 1
        
        #周回判定
        if self.current_wp > 12:
            rospy.loginfo('model_' + str(self.model_num) + ' success')
            self.is_finished = True
            self.is_success = True

        # 衝突判定（一定ステップごと）
        if self.step_counter % self.loop_check_interval == 0:
            self.collision()

        #終了判定
        if self.is_finished == True:
            self.write_score()
            os.system('killall roslaunch')
            sys.exit()
        
        # 走行時の視覚画像を表示 CPU使用率削減のためコメントアウト
        # cv2.imshow("Original Image", self.cv_image)
        # cv2.waitKey(1)

        # 画像を学習時と同じサイズ (48, 64, 3) にリサイズ
        img = cv2.resize(self.cv_image, (64, 48), interpolation=cv2.INTER_AREA)

        # 画像をモデルに入力して、予測された角速度を取得
        target_action = self.dl.act(img)

        # ロボットの移動指令を作成
        self.vel.linear.x = 0.2  # 直進速度は固定0.2
        self.vel.angular.z = target_action  # モデルから出力された角速度を使用
        print(f"model: {self.model_num}, angular: {self.vel.angular.z}")

        # パブリッシュ
        self.nav_pub.publish(self.vel)


if __name__ == '__main__':
    rg = nav_cloning_node()
    DURATION = 0.2
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        rg.loop()
        r.sleep()
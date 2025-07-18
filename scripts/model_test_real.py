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
        
        self.center_sub = rospy.Subscriber("/camera/lane1/center/rgb/usb_cam/image_raw", Image, self.callback)
        
        
        self.nav_pub = rospy.Publisher('/icart_mini/cmd_vel', Twist, queue_size=10)
        
        self.cv_image = np.zeros((240, 320, 3), np.uint8)
        self.vel = Twist()
        self.pro = "20250517_12:49:45"  # モデルファイル
        self.model_num = rospy.get_param("/nav_cloning_node/model_num", "1")
        self.load_path = roslib.packages.get_pkg_dir('nav_cloning') + f'/data/model/{self.pro}/model{self.model_num}.pt'
        self.score = roslib.packages.get_pkg_dir('nav_cloning') + f'/data/score/{self.pro}.csv'
        self.dl.load(self.load_path)
        print(self.load_path)
        
        self.step_counter = 0
        
    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def loop(self):
        # 画像サイズが正しいか確認
        if self.cv_image.shape != (240, 320, 3):
            rospy.logwarn(f"画像サイズが想定と異なります: {self.cv_image.shape}")
            return

        #ステップ更新
        self.step_counter += 1
        
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
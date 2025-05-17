#!/usr/bin/env python3
from __future__ import print_function

import rospy
import cv2
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_cloning_pytorch import deep_learning  # deep_learningをインポート
from skimage.transform import resize
from geometry_msgs.msg import Twist
import numpy as np
import roslib
import copy

class nav_cloning_node:
    def __init__(self):
        rospy.init_node('nav_cloning_node', anonymous=True)
        self.num = rospy.get_param("/nav_cloning_node/num", "1")
        self.action_num = 1
        self.dl = deep_learning(n_action=self.action_num)  # deep_learningクラスを初期化
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/lane1/center/rgb/image_raw", Image, self.callback)
        #self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.vel_sub = rospy.Subscriber("/nav_vel", Twist, self.callback_vel)
        self.action_pub = rospy.Publisher("action", Int8, queue_size=1)
        self.nav_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.cv_image = np.zeros((520, 694, 3), np.uint8)
        self.vel = Twist()
        self.learning = False
        self.pro = "20250517_12:49:45"  # モデルファイル run_collect→"20250419_23:59:33" set"20250506_00:14:39" 9cam_run "20250517_12:49:45"
        self.load_path = roslib.packages.get_pkg_dir('nav_cloning') + f'/data/model/{self.pro}/model1.pt'
        
        if self.learning == False:
            print(self.load_path)
            self.dl.load(self.load_path)  # モデルをロード
        
        
    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def callback_vel(self, data):
        self.vel = data

    def loop(self):
        if self.cv_image.size != 640 * 480 * 3: #1280 * 640 * 3:  # 画像サイズが正しいか確認
            return

        # 元の画像を表示（デバッグ用）
        cv2.imshow("Original Image", self.cv_image)
        cv2.waitKey(1)

        # 画像を学習時と同じサイズ (48, 64, 3) にリサイズ
        img = cv2.resize(self.cv_image, (64, 48), interpolation=cv2.INTER_AREA)

        if self.learning == False:
            # 画像をモデルに入力して、予測された角速度を取得
            target_action = self.dl.act(img)

            # ロボットの移動指令を作成
            self.vel.linear.x = 0.2  # 直進速度は固定0.2
            self.vel.angular.z = target_action  # モデルから出力された角速度を使用

            # パブリッシュ
            self.nav_pub.publish(self.vel)

        

if __name__ == '__main__':
    rg = nav_cloning_node()
    DURATION = 0.2
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        rg.loop()
        r.sleep()
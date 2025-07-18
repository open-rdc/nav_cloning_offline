#!/usr/bin/env python3
from __future__ import print_function
import roslib
roslib.load_manifest('nav_cloning')
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from std_srvs.srv import SetBool, SetBoolResponse
import csv
import os
import time
import numpy as np
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), 'pytorch'))
from nav_cloning_pytorch import *

class cource_following_learning_node:
    def __init__(self):
        rospy.init_node('cource_following_learning_node', anonymous=True)
        self.action_num = rospy.get_param("/LiDAR_based_learning_node/action_num", 1)
        print("action_num: " + str(self.action_num))
        self.bridge = CvBridge()

        # カメラデータ購読
        self.center_sub = rospy.Subscriber("/camera/lane1/center/rgb/usb_cam/image_raw", Image, self.callback)
        self.left_sub = rospy.Subscriber("/camera/lane2/center/rgb/usb_cam/image_raw", Image, self.callback_left_camera)
        self.right_sub = rospy.Subscriber("/camera/lane3/center/rgb/usb_cam/image_raw", Image, self.callback_right_camera)
 
        self.vel_sub = rospy.Subscriber("/icart_mini/cmd_vel", Twist, self.callback_vel, queue_size=10)
        
        self.cv_image = np.zeros((520, 694, 3), np.uint8)
        self.cv_left_image = np.zeros((520, 694, 3), np.uint8)
        self.cv_right_image = np.zeros((520, 694, 3), np.uint8)

        self.vel = Twist()
        self.action = 0.0
        self.episode = 0
        self.save_img_no = 0

        self.start_time = time.strftime("%Y%m%d_%H:%M:%S")
        self.path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/'

        os.makedirs(self.path + "img/" + self.start_time)
        os.makedirs(self.path + "ang/" + self.start_time)

    def capture_img(self):
        try:
            # 画像リサイズ
            self.resize_left_img = cv2.resize(self.cv_left_image, (64, 48), interpolation=cv2.INTER_AREA)
            self.resize_img = cv2.resize(self.cv_image, (64, 48), interpolation=cv2.INTER_AREA)
            self.resize_right_img = cv2.resize(self.cv_right_image, (64, 48), interpolation=cv2.INTER_AREA)

            # 保存
            save_dir = self.path + "img/" + self.start_time + "/"
            cv2.imwrite(save_dir + "lane1_center_" + str(self.save_img_no) + ".jpg", self.resize_img)
            cv2.imwrite(save_dir + "lane2_center_" + str(self.save_img_no) + ".jpg", self.resize_left_img)
            cv2.imwrite(save_dir + "lane3_center_" + str(self.save_img_no) + ".jpg", self.resize_right_img)


            print('Saved image number:', self.save_img_no)
        except:
            print('Failed to save image')

    def capture_ang(self):
        line = [str(self.save_img_no), str(self.action)]
        with open(self.path + "ang/" + self.start_time + '/ang.csv', 'a') as f:
            writer = csv.writer(f, lineterminator='\n')
            writer.writerow(line)

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def callback_left_camera(self, data):
        try:
            self.cv_left_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def callback_right_camera(self, data):
        try:
            self.cv_right_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def callback_vel(self, data):
        self.vel = data
        self.action = self.vel.angular.z

    def loop(self):
        # 画像がまだ届いていない場合はスキップ
        if self.cv_image.size != 320 * 240 * 3:
            return
        if self.cv_left_image.size != 320 * 240 * 3:
            return
        if self.cv_right_image.size != 320 * 240 * 3:
            return

        # 保存
        self.capture_img()
        self.capture_ang()
        self.save_img_no += 1

        # 学習終了条件（任意）
        if self.episode == 4000:
            self.learning = False

if __name__ == '__main__':
    rg = cource_following_learning_node()
    DURATION = 0.5  # 保存間隔（秒）
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        rg.loop()
        r.sleep()

#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
import csv
import os
import time
import numpy as np
import math
import roslib

class CourceFollowingLearningNode:
    def __init__(self):
        rospy.init_node('cource_following_learning_node', anonymous=True)
        self.bridge = CvBridge()

        self.num_cameras = 9
        self.cv_images = [np.zeros((520, 694, 3), np.uint8) for _ in range(self.num_cameras)]
        self.resized_images = [None for _ in range(self.num_cameras)]

        # カメラのサブスクライバ初期化
        img_types = ["center", "left", "right"]
        for i in range(1, 4):  # lane1 ~ lane3
            for j, img_type in enumerate(img_types):  # j: 0,1,2
                topic_name = f"/camera/lane{i}/{img_type}/rgb/image_raw"
                cam_index = (i - 1) * 3 + j  # 0〜8のインデックス
                rospy.Subscriber(topic_name, Image, self.make_image_callback(cam_index))

        self.vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.callback_vel, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.callback_odom)
        self.path_sub = rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.callback_path)

        self.action = 0.0
        self.episode = 0
        self.vel = Twist()
        self.path_pose = Path()
        self.learning = True
        self.flag = False

        self.pose_x = 0
        self.pose_y = 0
        self.old_pose_x = 0
        self.old_pose_y = 0

        self.save_img_no = 0
        self.start_time = time.strftime("%Y%m%d_%H:%M:%S")
        self.path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/'
        os.makedirs(self.path + "img/" + self.start_time)
        os.makedirs(self.path + "ang/" + self.start_time)

    def make_image_callback(self, index):
        def callback(data):
            try:
                self.cv_images[index] = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                rospy.logerr(e)
        return callback

    def callback_vel(self, data):
        self.vel = data
        self.action = data.angular.z

    def callback_odom(self, data):
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y

    def callback_path(self, data):
        self.path_pose = data

    def check_distance(self):
        distance = math.sqrt((self.pose_x - self.old_pose_x)**2 + (self.pose_y - self.old_pose_y)**2)
        if distance >= 0.05:
            self.old_pose_x = self.pose_x
            self.old_pose_y = self.pose_y
            self.flag = True

    def capture_img(self):
        try:
            for i in range(self.num_cameras):
                # lane番号とimg_typeを逆算
                lane = i // 3 + 1  # 1〜3
                img_type = ['center', 'left', 'right'][i % 3]

                filename = self.path + f"img/{self.start_time}/lane{lane}_{img_type}_{self.save_img_no}.jpg"
                cv2.imwrite(filename, self.resized_images[i])

            rospy.loginfo(f"Saved image set #{self.save_img_no}")
        except Exception as e:
            rospy.logwarn(f"Failed to save images: {e}")

    def capture_ang(self):
        line = [str(self.save_img_no), str(self.action)]
        with open(self.path + f"ang/{self.start_time}/ang.csv", 'a') as f:
            writer = csv.writer(f, lineterminator='\n')
            writer.writerow(line)

    def loop(self):
        self.check_distance()
        if self.flag:
            for i in range(self.num_cameras):
                self.resized_images[i] = cv2.resize(self.cv_images[i], (64, 48), interpolation=cv2.INTER_AREA)
            self.capture_img()
            self.capture_ang()
            self.save_img_no += 1
            self.flag = False

        for img in self.cv_images:
            if img.size != 694 * 520 * 3:
                return

        if self.episode == 4000:
            self.learning = False

if __name__ == '__main__':
    node = CourceFollowingLearningNode()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        node.loop()
        rate.sleep()

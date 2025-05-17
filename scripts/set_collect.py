#!/usr/bin/env python3
from __future__ import print_function
import roslib
roslib.load_manifest('nav_cloning')
import rospy
import cv2
from geometry_msgs.msg import PoseWithCovarianceStamped,Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_cloning_pytorch import *
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

import math
import tf.transformations

from std_srvs.srv import SetBool, SetBoolResponse
import csv
import os
import time
import sys
import datetime


class GoalAngleSimulator:
    def __init__(self):
        rospy.init_node('goal_angle_simulator', anonymous=True)

        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        self.amcl_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

        rospy.Subscriber('/nav_vel', Twist, self.nav_vel_callback)
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        
        # 画像関連の準備
        self.bridge = CvBridge()
        self.cv_image = None  # カメラ画像を格納する変数

        # 角速度の保存用
        self.latest_ang_vel = 0.0

        # CSV読み込み
        csv_path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/path/00_02_fix.csv'
        with open(csv_path, 'r') as f:
            self.pos_list = [line.strip().split(',') for line in f]

        self.offset_x = 10.71378
        self.offset_y = 17.17456
        self.goal_offset = 10  # Nステップ後をゴールに

        self.image_save_no = 0  # 画像保存用の番号

        # 保存先ディレクトリ作成
        self.start_time = time.strftime("%Y%m%d_%H:%M:%S")
        
        self.image_save_path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/img/'  + self.start_time
        self.ang_save_path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/ang/'  + self.start_time
        os.makedirs(self.image_save_path)
        os.makedirs(self.ang_save_path)
        
        # 角速度を保存するCSVファイルの準備
        self.csv_file = open(self.ang_save_path + '/ang.csv', 'w')
        self.csv_writer = csv.writer(self.csv_file)
        
        self.run()

    def nav_vel_callback(self, msg):
        self.latest_ang_vel = msg.angular.z
        print(f"[angular.z] = {self.latest_ang_vel:.4f}")

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Image conversion failed: {e}")

    def move_robot_pose(self, x, y, theta):
        # Gazebo
        state = ModelState()
        state.model_name = 'turtlebot3'
        state.pose.position.x = x
        state.pose.position.y = y
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        state.pose.orientation.x = quat[0]
        state.pose.orientation.y = quat[1]
        state.pose.orientation.z = quat[2]
        state.pose.orientation.w = quat[3]

        try:
            self.set_state_srv(state)
        except rospy.ServiceException as e:
            rospy.logerr("SetModelState failed: %s" % e)

        # AMCL
        amcl = PoseWithCovarianceStamped()
        amcl.header.stamp = rospy.Time.now()
        amcl.header.frame_id = "map"
        amcl.pose.pose.position.x = x - self.offset_x
        amcl.pose.pose.position.y = y - self.offset_y
        amcl.pose.pose.orientation.x = quat[0]
        amcl.pose.pose.orientation.y = quat[1]
        amcl.pose.pose.orientation.z = quat[2]
        amcl.pose.pose.orientation.w = quat[3]
        amcl.pose.covariance[-1] = 0.01
        self.amcl_pose_pub.publish(amcl)

    def publish_goal(self, x, y):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = x - self.offset_x
        goal.pose.position.y = y - self.offset_y
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

    def save_image(self, count, offset_ang):
        # 画像を保存する関数
        if self.cv_image is not None:
            filename = f"{self.image_save_path}/center{count}_{offset_ang}.jpg"
            try:
                self.resize_img = cv2.resize(self.cv_image, (64, 48), interpolation=cv2.INTER_AREA)
                cv2.imwrite(filename, self.resize_img)
            except Exception as e:
                rospy.logwarn(f"Failed to save image: {e}")
            return filename
        else:
            rospy.logwarn("No image available to save.")
            return None

    def save_to_csv(self, step, ang_vel, image_filename):
        # CSVにデータを保存する関数
        self.csv_writer.writerow([step, ang_vel, image_filename])

    def run(self):
        rate = rospy.Rate(0.5)  # 2秒ごと（AMCLやmove_baseが安定するように）

        for i in range(len(self.pos_list) - self.goal_offset):
            
            for offset_ang in [-5, 0, +5]:
            
                # 現在位置
                cur = self.pos_list[i]
                x, y, theta = float(cur[1]), float(cur[2]), float(cur[3])
                theta += math.radians(offset_ang)  # ← ここで傾ける
                self.move_robot_pose(x, y, theta)

                rospy.sleep(1.0)  # AMCL反映のため

                # ゴール位置（中央レーンのみをゴールに）
                for j in range(i + 0, len(self.pos_list)):
                    if (j - i) >= self.goal_offset and (j % 3 == 0):  # 中央レーン
                        gx, gy = float(self.pos_list[j][1]), float(self.pos_list[j][2])
                        self.publish_goal(gx, gy)
                        break

                rospy.sleep(2.0)  # move_baseがnav_velを出すのを待つ

                # 画像保存（offset_angを使ってファイル名を指定）
                image_filename = self.save_image(self.image_save_no, offset_ang)

                # CSVに保存
                self.save_to_csv(i, self.latest_ang_vel, image_filename)

                print(f"[Step {i}] offset_ang: {offset_ang}°, 角速度: {self.latest_ang_vel:.4f} rad/s\n")
                self.image_save_no += 1

                rate.sleep()

        # CSVファイルを閉じる
        self.csv_file.close()
    
if __name__ == '__main__':
    GoalAngleSimulator()

#!/usr/bin/env python3
from __future__ import print_function
import roslib
roslib.load_manifest('nav_cloning')
import rospy
import cv2
from geometry_msgs.msg import PoseWithCovarianceStamped,Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
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
        self.nav_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.image_sub = rospy.Subscriber("/camera/lane1/center/rgb/image_raw", Image, self.callback)
        
        # CSV読み込み
        csv_path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/path/00_02_fix.csv'
        with open(csv_path, 'r') as f:
            self.pos_list = [line.strip().split(',') for line in f]

        self.initial_pose_x = -10.71378
        self.initial_pose_y = -17.17456
        
        self.image_save_no = 0  # 画像保存用の番号
        
        self.run()

    def callback(self):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

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
        amcl.pose.pose.position.x = x + self.initial_pose_x
        amcl.pose.pose.position.y = y + self.initial_pose_y
        amcl.pose.pose.orientation.x = quat[0]
        amcl.pose.pose.orientation.y = quat[1]
        amcl.pose.pose.orientation.z = quat[2]
        amcl.pose.pose.orientation.w = quat[3]
        amcl.pose.covariance[-1] = 0.01
        self.amcl_pose_pub.publish(amcl)

    
    def run(self):
        rate = rospy.Rate(0.5)  # 2秒ごと（AMCLが安定するように）

        for i in range(len(self.pos_list)):
            
            # 移動位置
            cur = self.pos_list[i]
            x, y, theta = float(cur[1]), float(cur[2]), float(cur[3])
            self.move_robot_pose(x, y, theta)

            rospy.sleep(1.0)  # AMCL反映のため

            rate.sleep()

if __name__ == '__main__':
    GoalAngleSimulator()

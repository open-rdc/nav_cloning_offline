#!/usr/bin/env python3
from __future__ import print_function
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import tf.transformations
import numpy as np
import roslib
import csv
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../pytorch'))
from nav_cloning_pytorch import deep_learning


class PathRecoveryEvaluator:
    def __init__(self):
        rospy.init_node('path_recovery_evaluator', anonymous=True)

        self.cv_image = np.zeros((520, 694, 3), np.uint8)
        self.bridge = CvBridge()
        self.vel = Twist()

        self.action_num = 1
        self.pro = rospy.get_param("/nav_cloning_node/model_dir", "20250517_12:49:45")
        self.model_num = rospy.get_param("/nav_cloning_node/model_num", "1")
        self.initial_pose_x = -10.71378
        self.initial_pose_y = -17.17456

        self.dl = deep_learning(n_action=self.action_num)
        load_path = roslib.packages.get_pkg_dir('nav_cloning') + f'/data/model/{self.pro}/model{self.model_num}.pt'
        self.dl.load(load_path)
        rospy.loginfo(f"Loaded model from: {load_path}")

        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.amcl_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.nav_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.image_sub = rospy.Subscriber("/camera/lane1/center/rgb/image_raw", Image, self.callback)

        csv_path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/path/00_02_fix.csv'
        with open(csv_path, 'r') as f:
            self.pos_list = [line.strip().split(',') for line in f]

        self.run()

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"cv_bridge error: {e}")

    def move_robot_pose(self, x, y, theta):
        state = ModelState()
        state.model_name = 'turtlebot3'
        state.pose.position.x = x
        state.pose.position.y = y
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w = quat

        try:
            self.set_state_srv(state)
        except rospy.ServiceException as e:
            rospy.logerr(f"SetModelState failed: {e}")

        amcl = PoseWithCovarianceStamped()
        amcl.header.stamp = rospy.Time.now()
        amcl.header.frame_id = "map"
        amcl.pose.pose.position.x = x + self.initial_pose_x
        amcl.pose.pose.position.y = y + self.initial_pose_y
        amcl.pose.pose.orientation.x, amcl.pose.pose.orientation.y, amcl.pose.pose.orientation.z, amcl.pose.pose.orientation.w = quat
        amcl.pose.covariance[-1] = 0.01
        self.amcl_pose_pub.publish(amcl)

    def inference_with_models(self):
        img = cv2.resize(self.cv_image, (64, 48))
        target_action = self.dl.act(img)
        self.vel.linear.x = 0.2
        self.vel.angular.z = target_action
        self.nav_pub.publish(self.vel)

    def run(self):
        rate = rospy.Rate(10)
        for i, cur in enumerate(self.pos_list):
            if self.cv_image.shape != (480, 640, 3):
                rospy.loginfo("cv_image size is unexpected, skipping.")
                continue

            x, y, theta = float(cur[1]), float(cur[2]), float(cur[3])
            self.move_robot_pose(x, y, theta)

            for _ in range(20):
                self.inference_with_models()
                rate.sleep()


if __name__ == '__main__':
    try:
        PathRecoveryEvaluator()
    except rospy.ROSInterruptException:
        pass

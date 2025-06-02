#!/usr/bin/env python3

from __future__ import print_function
import roslib
roslib.load_manifest('nav_cloning')
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import csv
import os
import math
from tf.transformations import euler_from_quaternion


class PathCollectorNode:
    def __init__(self):
        rospy.init_node('path_collector_node', anonymous=True)

        self.vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.callback_vel)
        
        pkg_dir = roslib.packages.get_pkg_dir('nav_cloning')
        self.path_dir = os.path.join(pkg_dir, 'data', 'path')
        os.makedirs(self.path_dir, exist_ok=True)

        self.path_file = os.path.join(self.path_dir, 'path.csv')
        with open(self.path_file, 'w', newline='') as f:
            writer = csv.writer(f)
            
        self.path_no = 0
        self.vel = Twist()
        self.vel_angular = 0.0

        self.latest_pose = None
        self.latest_yaw = 0.0

        self.pose_sub = rospy.Subscriber("/tracker", Odometry, self.callback_pose)

    def callback_pose(self, data):
        self.latest_pose = data.pose.pose
        # クォータニオンからyawを計算
        orientation_q = self.latest_pose.orientation
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w)
        _, _, yaw = euler_from_quaternion(quaternion)
        self.latest_yaw = yaw

    def callback_vel(self, data):
        self.vel = data
        self.vel_angular = data.angular.z

    def loop(self):
        # 速度指令を転送
        self.vel.angular.z = self.vel_angular
        
        if self.latest_pose is not None:
            x = self.latest_pose.position.x
            y = self.latest_pose.position.y
            yaw = self.latest_yaw

            # 左右のオフセット距離[m]
            offsets = [0.1, 0.2, 0.3]

            with open(self.path_file, 'a', newline='') as f:
                writer = csv.writer(f)
                # 左側の点（yaw + π/2方向）
                for off in offsets:
                    offset_x = x + off * math.cos(yaw + math.pi / 2)
                    offset_y = y + off * math.sin(yaw + math.pi / 2)
                    writer.writerow([self.path_no, offset_x, offset_y, yaw])
                    self.path_no += 1

                # 右側の点（yaw - π/2方向）
                for off in offsets:
                    offset_x = x + off * math.cos(yaw - math.pi / 2)
                    offset_y = y + off * math.sin(yaw - math.pi / 2)
                    writer.writerow([self.path_no, offset_x, offset_y, yaw])
                    self.path_no += 1


if __name__ == '__main__':
    pc = PathCollectorNode()
    rate = rospy.Rate(10/25)  # 10 Hz
    while not rospy.is_shutdown():
        pc.loop()
        rate.sleep()

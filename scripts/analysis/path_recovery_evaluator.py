#!/usr/bin/env python3
import rospy
import math
import roslib
roslib.load_manifest('nav_cloning')
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import tf.transformations
import time
import csv

class SimpleMover:
    def __init__(self):
        rospy.init_node('simple_mover', anonymous=True)
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # CSVロード
        csv_path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/path/00_02_fix.csv'
        with open(csv_path, 'r') as f:
            self.pos_list = [line.strip().split(',') for line in f]

        self.run()

    def move(self, x, y, theta):
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
            rospy.logerr(f"SetModelState failed: {e}")

    def run(self):
        for i, cur in enumerate(self.pos_list):
            x, y, theta = float(cur[1]), float(cur[2]), float(cur[3])
            for offset in [-5, 0, +5]:
                self.move(x, y, theta + math.radians(offset))
                rospy.loginfo(f"Moved to: {x:.2f}, {y:.2f}, {theta + math.radians(offset):.2f} rad")
                rospy.sleep(2)

if __name__ == '__main__':
    SimpleMover()

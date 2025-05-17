#!/usr/bin/env python3
from __future__ import print_function
import roslib
roslib.load_manifest('nav_cloning')
import rospy
import csv
import math

class calc_capture_pos:
    def __init__(self):
        rospy.init_node('calc_capture_pos_node', anonymous=True)
        self.path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/path/'

    def calc_pos(self):
        with open(self.path + 'amcl_pose_02.csv', 'a') as fw:
            writer = csv.writer(fw, lineterminator='\n')
            with open(self.path + 'amcl_pose.csv', 'r') as fr:
                for row in csv.reader(fr):
                    str_x, str_y = row
                    x, y = float(str_x), float(str_y)
                    x0, y0 = x, y
                    angle = math.atan2(y - y0, x - x0)
                    for dy in [0.2]:
                        line = [str(x-dy*math.sin(angle)), str(y+dy*math.cos(angle))]
                        writer.writerow(line)
                    x0, y0 = x, y

if __name__ == '__main__':
    ccp = calc_capture_pos()
    ccp.calc_pos()


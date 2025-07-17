#!/usr/bin/env python3
from __future__ import print_function
import csv
import roslib
import rospy
import cv2
import sys

class target_path:
    def __init__(self):
        rospy.init_node('target_path', anonymous=True)
        self.image = cv2.imread(roslib.packages.get_pkg_dir('nav_cloning')+'/maps/willowgarage.pgm')
        self.image_resize = cv2.resize(self.image, (600, 600))
        self.file_path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/path/path_willow_shirasu.csv'
        self.save_path = "/home/yuya/nav_cloning_offline_ws/src/nav_cloning/data/result/analysis/pos_test,pgm"
        self.image_name = ""
        self.path_x = []
        self.path_y = []
        self.pos_x = 0
        self.pos_y = 0
        self.old_pos_x = self.pos_x
        self.old_pos_y = self.pos_y
        self.count = 0

        with open(self.file_path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                self.path_x.append(row[0])
                self.path_y.append(row[1])

    # def draw_circle(self, pos_x, pos_y):
    #     self.vis_x = 205 + int(pos_x * 10.7)
    #     self.vis_y = 325 + int(pos_y * 13.5) * (-1)
    #     cv2.circle(self.image_resize, (self.vis_x, self.vis_y), 2, (0, 0, 255), thickness = 3)
    #     cv2.circle(self.image_resize, (205 + int(self.pos_x * 10.7), 325 + int(self.pos_y * 13.5) * (-1)), 2, (0, 0, 255), thickness = 3)
    
    # def draw_line(self, pos_x, pos_y, old_pos_x, old_pos_y):
    #     self.vis_x = 205 + int(pos_x * 10.7)
    #     self.vis_y = 325 + int(pos_y * 13.5) * (-1)
    #     self.old_vis_x = 205 + int(old_pos_x * 10.7)
    #     self.old_vis_y = 325 + int(old_pos_y * 13.5) * (-1)
    #     cv2.line(self.image_resize, (self.vis_x, self.vis_y), (self.old_vis_x, self.old_vis_y), (255, 0, 0), thickness=3)
    #     cv2.line(self.image_resize, (205 + int(pos_x * 10.7), 325 + int(pos_y * 13.5) * (-1)), (205 + int(old_pos_x * 10.7), 325 + int(old_pos_y * 13.5) * (-1)), (255, 0, 0), thickness=3)

    def draw_circle_line(self, pos_x, pos_y, old_pos_x, old_pos_y, count):
        self.vis_x = 205 + int(pos_x * 10.7)
        self.vis_y = 325 + int(pos_y * 13.5) * (-1)
        self.old_vis_x = 205 + int(old_pos_x * 10.7)
        self.old_vis_y = 325 + int(old_pos_y * 13.5) * (-1)
        cv2.circle(self.image_resize, (self.vis_x, self.vis_y), 2, (0, 0, 255), thickness = 3)
        if count >= 1:
            cv2.line(self.image_resize, (self.vis_x, self.vis_y), (self.old_vis_x, self.old_vis_y), (255, 0, 0), thickness=3)

    def loop(self):
        self.pos_x = float(self.path_x[self.count])
        self.pos_y = float(self.path_y[self.count])
        # print("pos_x", self.pos_x)
        self.draw_circle_line(self.pos_x, self.pos_y, self.old_pos_x, self.old_pos_y, self.count)
        self.crop_img = self.image_resize.copy()
        self.crop_img = self.crop_img[301:600, 0:600]
        cv2.imshow("target_path", self.crop_img)
        cv2.waitKey(1)
        self.old_pos_x = self.pos_x
        self.old_pos_y = self.pos_y
        # print("old_pos_x", self.old_pos_x)
        self.count += 1
        print(self.count)
        if self.count == len(self.path_x):
            cv2.imwrite(self.save_path + self.image_name, self.crop_img)
            print("save image -> " + self.save_path)
            sys.exit()

if __name__ == '__main__':
    rg = target_path()
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        rg.loop()
        r.sleep()

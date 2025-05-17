#!/usr/bin/env python3
from __future__ import print_function
import roslib
roslib.load_manifest('nav_cloning')
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_cloning_pytorch import *
from skimage.transform import resize
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int8
from std_srvs.srv import Trigger
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
from std_srvs.srv import SetBool, SetBoolResponse
import csv
import os
import time
import copy
from nav_msgs.msg import Odometry
import math

class cource_following_learning_node:
    def __init__(self):
        rospy.init_node('cource_following_learning_node', anonymous=True)
        self.action_num = rospy.get_param("/LiDAR_based_learning_node/action_num", 1)
        print("action_num: " + str(self.action_num))
        self.bridge = CvBridge()

        #カメラからのデータをサブスクライブ
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.image_left_sub = rospy.Subscriber("/camera_left/rgb/image_raw", Image, self.callback_left_camera)
        self.image_right_sub = rospy.Subscriber("/camera_right/rgb/image_raw", Image, self.callback_right_camera)

        self.vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.callback_vel, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.callback_odom)
        self.path_sub = rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.callback_path)
        self.min_distance = 0.0
        self.action = 0.0
        self.episode = 0
        self.vel = Twist()
        self.path_pose = PoseArray()
        self.cv_image = np.zeros((520,694,3), np.uint8)
        self.cv_left_image = np.zeros((520,694,3), np.uint8)
        self.cv_right_image = np.zeros((520,694,3), np.uint8)
        self.learning = True
        self.select_dl = False
        self.start_time = time.strftime("%Y%m%d_%H:%M:%S")
        self.path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/result/'
        self.save_path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/model/'
        self.previous_reset_time = 0
        self.start_time_s = rospy.get_time()
        self.save_img_no = 0
        self.save_img_left_no = 1
        self.save_img_center_no = 0
        self.save_img_right_no = -1
        self.path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/'
        self.pose_x = 0
        self.pose_y = 0
        self.old_pose_x = 0
        self.old_pose_y = 0
        self.current_pose_x = 0
        self.current_pose_y = 0
        self.flag = False
        os.makedirs(self.path + "img/" + self.start_time)
        os.makedirs(self.path + "ang/" + self.start_time)
        self.path_pose_x = 0
        self.path_pose_y = 0
        self.path_no = 0

    #キャプチャと保存
    def capture_img(self):
            Flag = True
            try:
                #カメラ画像を保存
                cv2.imwrite(self.path + "img/" + self.start_time + "/left" + str(self.save_img_no) + ".jpg", self.resize_left_img)
                cv2.imwrite(self.path + "img/" + self.start_time + "/center" + str(self.save_img_no) + ".jpg", self.resize_img)
                cv2.imwrite(self.path + "img/" + self.start_time + "/right" + str(self.save_img_no) + ".jpg", self.resize_right_img)
                
            except:
                print('Not save image')
                Flag = False
            finally:
                if Flag:
                    print('Save image Number:', self.save_img_no)

    #角速度保存
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

    def callback_path(self, data):
        self.path_pose = data

    def check_distance(self):
        distance = math.sqrt((self.pose_x - self.old_pose_x)**2 + (self.pose_y - self.old_pose_y)**2)
        print("distance : ", distance)
        if distance >= 0.05:
            self.old_pose_x = self.pose_x
            self.old_pose_y = self.pose_y
            self.flag = True

    def callback_odom(self, data):
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y      

    #移動の角速度を格納
    def callback_vel(self, data):
        self.vel = data
        self.action = self.vel.angular.z

    def callback_dl_training(self, data):
        resp = SetBoolResponse()
        self.learning = data.data
        resp.message = "Training: " + str(self.learning)
        resp.success = True
        return resp

    def loop(self):
        self.check_distance()
        if self.flag:

            # 画像リサイズ            
            self.resize_left_img = cv2.resize(self.cv_left_image, (64, 48), interpolation=cv2.INTER_AREA)
            self.resize_img = cv2.resize(self.cv_image, (64, 48), interpolation=cv2.INTER_AREA)
            self.resize_right_img = cv2.resize(self.cv_right_image, (64, 48), interpolation=cv2.INTER_AREA)
    
            self.capture_img()
            self.capture_ang()
            self.save_img_no += 1
            self.flag = False
        
        if self.cv_image.size != 694 * 520 * 3:
            return
        if self.cv_left_image.size != 694 * 520 * 3:
            return
        if self.cv_right_image.size != 694 * 520 * 3:
            return

        if self.episode == 4000:
            self.learning = False

if __name__ == '__main__':
    rg = cource_following_learning_node()
    DURATION = 0.1
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        rg.loop()
        r.sleep()

#!/usr/bin/env python3
import roslib.packages
import rospy
import roslib
import os
import time
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_srvs.srv import SetBool, SetBoolResponse

class capture_img_node:
    def __init__(self):
        rospy.init_node('capture_img_node', anonymous=True)
        self.bridge = CvBridge()
        # self.img_sub = rospy.Subscriber('/image/mercator', Image, self.img_callback)
        self.img_sub = rospy.Subscriber('/camera/rgb/image_raw_front', Image, self.img_callback)
        rospy.Service('capture_img', SetBool, self.capture_srv)
        # self.cv_image = np.zeros((640,1280,3), np.uint8)
        self.cv_image = np.zeros((480,640,3), np.uint8)
        self.path = os.path.join(roslib.packages.get_pkg_dir('nav_cloning'), 'data')
        self.start_time = time.strftime("%Y%m%d_%H:%M:%S")
        self.save_img_no = 0
        os.makedirs(os.path.join(self.path, self.start_time))
        rospy.spin()

    def img_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def capture_srv(self, req):
        if req.data:
            self.capture()
            return SetBoolResponse(True, "Image captured successfully")
        else:
            return SetBoolResponse(False, "Invalid request")
        
    def capture(self):
        Flag = True
        try:
            # rospy.wait_for_service("/capture_img")
            image_path = os.path.join(self.path, self.start_time, f"test_{self.save_img_no}.png")
            cv2.imwrite(image_path, self.cv_image)
            self.save_img_no += 1
        except Exception as e:
            print(f"Failed to save image: {e}")
            Flag = False
        finally:
            if Flag:
                print("caputure_img successfully")

if __name__ == '__main__':
    capture_img_node()
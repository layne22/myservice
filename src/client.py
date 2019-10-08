#!/usr/bin/env python 

import roslib
roslib.load_manifest('myservice')
import rospy

# from myservice.srv import WordCount
from myservice.srv import trans_count
import sys

import numpy as np
import scipy.misc

import rospy
import cv2
from distutils.version import LooseVersion
# if LooseVersion(cv2.__version__).version[0] == 2

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

from scipy.misc import imresize, imsave, imshow
import time
import os


class ImageConverter:
    def __init__(self):
        self.imagetopic = "/camera/rgb/image_raw"
        self.depthtopic = "/camera/depth/image_raw"
        self.pointstopic = "/camera/depth_registered/points"
        self.irtopic = '/camera/ir/image'
        self.bridge = CvBridge()
        self.depth_color = None
        # self.image_color_sub = rospy.Subscriber(self.imagetopic, Image, self.image_callback)
        self.depth_sub = rospy.Subscriber(self.irtopic, Image, self.ir_callback)
        # self.points_cloud_sub = rospy.Subscriber(self.pointstopic, Image, self.points_callback)
        self.count = 0
        self.cwd = os.getcwd()
        # count = 0

    def image_callback(self, data):

        try:
            # data.encoding = "mono16"
            # cvImage = bridge.imgmsg_to_cv2(data, "mono8")
            img_color = self.bridge.imgmsg_to_cv2(data, "bgr8")  # mono16
            # print np.max((img_color/1000.0)*255)
            # img_color = (img_color/1000.0)*255
            img_color = scipy.misc.toimage(img_color, cmin=0.0, cmax=255.0)
            img_color = np.array(img_color)
            # print count
            # count_lock.acquire()
            # count += 1
            # print count
            t1 = time.time()
            cv2.imshow("Image window", img_color)
            k = cv2.waitKey(10)
            if k!=-1:
                # print '====================>', k
                print('====================>', k)
            if k == 32:
                img_name = "src/Images/img_raw%02d.png" % self.count
                cv2.imwrite(img_name, img_color)
                # cv2.imwrite("raw_data03/depth/depth_raw00" + str(num) + ".png", depth_inf)
                # imsave(""+str(num)+".png", img_color)
                self.send_for_srv()
                self.count += 1

            print("prediction duration: ", time.time()-t1)
        except CvBridgeError as e:
            print(e)
            cv2.destroyWindow("Image window")

    def ir_callback(self, data):

        try:
            # cvImage = bridge.imgmsg_to_cv2(data, "mono8")
            img_color = self.bridge.imgmsg_to_cv2(data, '16UC1')  # mono16
            # print np.max((img_color/1000.0)*255)
            # img_color = (img_color/1000.0)*255
            img_color = scipy.misc.toimage(img_color, cmin=0.0, cmax=255.0)
            img_color = np.array(img_color)

            cv2.imshow("Image window", img_color)
            k = cv2.waitKey(10)
            if k!=-1:
                # print '====================>', k
                print('====================>', k)

            if k == 32:
                img_name = "src/Images/img_raw%02d.png" % self.count
                cv2.imwrite(img_name, img_color)
                # cv2.imwrite("raw_data03/depth/depth_raw00" + str(num) + ".png", depth_inf)
                # imsave(""+str(num)+".png", img_color)
                self.send_for_srv()
                self.count += 1

            # print "prediction duration: ", time.time()-t1
        except CvBridgeError as e:
            print(e)
            cv2.destroyWindow("Image window")

    def depth_callback(self, data):
        global depth_inf
        try:
            self.depth_color = self.bridge.imgmsg_to_cv2(data, "16UC1")
            depth_inf[:, :, :] = self.depth_color
            # print depth_color.shape
            # cv2.imshow("Depth window", depth_color)
            k = cv2.waitKey(10)
            # count += 1
            # print "fuck:", k

        except CvBridgeError as e:
            print(e)
            cv2.destroyWindow("Depth window")

    def points_callback(self, data):
        global depth_inf
        try:
            self.points_cloud = self.bridge.imgmsg_to_cv2(data, "16UC1")
            # depth_inf[:, :, :] = self.depth_color
            print(depth_color.shape)
            # cv2.imshow("Depth window", depth_color)
            k = cv2.waitKey(10)
            # count += 1
            # print "fuck:", k

        except CvBridgeError as e:
            print(e)
            cv2.destroyWindow("Depth window")

    def send_for_srv(self):
        try:
            num_sender = rospy.ServiceProxy('robot_calibration', trans_count)

            # words = ''.join(sys.argv[1:])
            # print words
            send_return = num_sender(self.cwd, self.count)
            # word_count = word_counter(words)
            print('---------->', send_return.count, send_return.b)
        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)


if __name__ == "__main__":
    print("reciving......")
    rospy.init_node('saveImg_client', anonymous=True)
    ic = ImageConverter()
    rate = rospy.Rate(1)  # 10hz

    rospy.wait_for_service('robot_calibration')

    try:
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

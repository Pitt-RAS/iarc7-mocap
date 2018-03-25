#!/usr/bin/env python

import rospy
import roslib
import tf

from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel

import cv_bridge import CvBridge
import numpy as np


class mocap_driver():
    def __init__(self, publish_tf)
        #Turns Ros image into Cv msg => numpy array
        self.bridge = CvBridge()

        self.cam_model = PinholeCameraModel()

        rospy.Subscriber("camera_topic",CameraInfo,self.camera_callback)
        self.hasCameraInfo = False

        while not self.hasCameraInfo:
            print "waiting for camera info."
            rospy.sleep(0.5)

        rospy.Subscriber("depth_image",Image,self.depth_callback)

        rospy.Subscriber("rgb_image",Image,self.image_callback)

        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        self.publish_tf = publish_tf

    def motion_callback(self, ):

    def image_callback(self, image):
        image_cv = self.bridge.imgmsg_to_cv2(image, image.encoding)
        self.cv_image = image_cv

    def depth_callback(self, image):
        image_cv = self.bridge.imgmsg_to_cv2(image, image.encoding)
        image_cv2 = np.squeeze(np.array(image_cv, dtype=np.float32))
        self.depth_image = image_cv2

    def camera_callback(self, camera_info):
        if not self.hasCameraInfo:
            self.cam_model.fromCameraInfo(camera_info)
            self.camera_info = camera_info
            self.parent_frame = self.camera_info.header.frame_id
        self.hasCameraInfo = True

if __name__ = 'main':
    rospy.init_node('motion_capture')

    publish_tf = rospy.get_param('motion_capture/publish_tf')
    my_mocap = mocap_driver(publish_tf)

    rospy.spin()

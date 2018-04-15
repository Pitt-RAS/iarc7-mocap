#!/usr/bin/env python

import rospy
import roslib
import tf

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from image_geometry import PinholeCameraModel

from cv_bridge import CvBridge
import cv2
import numpy as np
from mocaprgb import mocap

class mocap_driver():
    def __init__(self, publish_tf):
        #Turns Ros image into Cv msg => numpy array
        self.bridge = CvBridge()

        self.cam_model = PinholeCameraModel()

        rospy.Subscriber("camera_topic",CameraInfo,self.camera_callback)
        self.hasCameraInfo = False

        while not self.hasCameraInfo and  not rospy.is_shutdown():
            print "waiting for camera info."
            rospy.sleep(0.5)

        rospy.Subscriber("point_cloud2",PointCloud2,self.pc2_callback)

        rospy.Subscriber("rgb_image",Image,self.image_callback)
        self.hasImage = False
        self.hasDepth = False
        while ((not self.hasImage) or (not self.hasDepth)) and (not rospy.is_shutdown()):
            print "waiting for Image."
            rospy.sleep(0.5)

        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        self.publish_tf = publish_tf

        self.mycap = mocap(self.camera_info,self.parent_frame,self.point_cloud2,
                self.rgb_image,self.cam_model,self.listener,self.broadcaster)

        while(not rospy.is_shutdown()):
            self.mycap.publish(self.rgb_image,self.point_cloud2)

        cv2.destroyAllWindows()

    def image_callback(self, image):
        image_rgb = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        self.rgb_image = image_rgb
        self.hasImage = True


    def pc2_callback(self, point_cloud2):
        self.point_cloud2 = point_cloud2
        self.hasDepth = True

    def camera_callback(self, camera_info):
        if not self.hasCameraInfo:
            self.cam_model.fromCameraInfo(camera_info)
            self.camera_info = camera_info
            self.parent_frame = self.camera_info.header.frame_id
        self.hasCameraInfo = True


if __name__ == '__main__':
    rospy.init_node('mocap_driver')

    publish_tf = rospy.get_param('motion_capture/publish_tf',True)
    my_mocap = mocap_driver(publish_tf)

    rospy.spin()

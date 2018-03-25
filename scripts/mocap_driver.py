#!/usr/bin/env python

import rospy
import roslib
import tf

from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel

from cv_bridge import CvBridge
import cv2
import numpy as np
from mocap import mocap

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

        rospy.Subscriber("depth_image",Image,self.depth_callback)

        rospy.Subscriber("rgb_image",Image,self.image_callback)
        self.hasImage = False
        self.hasDepth = False

        while not self.hasImage and not self.hasDepth and not rospy.is_shutdown():
            print "waiting for Image."
            rospy.sleep(0.5)

        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        self.publish_tf = publish_tf

        self.mycap = mocap(self.camera_info,self.parent_frame,self.depth_image,self.rgb_image,
        self.cam_model,self.listener,self.broadcaster)

        while(not rospy.is_shutdown()):
            self.mycap.extract_fg(self.rgb_image)
            self.mycap.label_filter(self.depth_image)
            if(self.mycap.validate()):
                self.mycap.publish()
        cv2.destroyAllWindows()

    def image_callback(self, image):
        image_rgb = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        self.rgb_image = image_rgb
        self.hasImage = True


    def depth_callback(self, image):
        image_cv = self.bridge.imgmsg_to_cv2(image, image.encoding)
        image_cv2 = np.squeeze(np.array(image_cv, dtype=np.float32))
        self.depth_image = image_cv2
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

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

        self.time = rospy.Time.now()

        self.publish_tf = publish_tf

        self.mycap = mocap(self.camera_info,self.parent_frame,self.point_cloud2,
                self.rgb_image,self.cam_model,self.listener,self.broadcaster)


        self.marker_array =[
        rospy.get_param('~frontmarker'),
        rospy.get_param('~rightmarker'),
        rospy.get_param('~backmarker'),
        rospy.get_param('~leftmarker')]

        while(not rospy.is_shutdown()):
            self.time = self.mycap.publish(self.rgb_image,self.point_cloud2)
            self.markers_to_tf(self.time)

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

    def markers_to_tf(self,time):
        markers = []
        trans = []
        rot = []
        for marker in self.marker_array:
            try :
                (translation,rotation) = self.listener.lookupTransform(marker, self.parent_frame, time)
                markers.append(marker)
                trans.append(np.array(translation))
                rot.append(np.array(rotation))
            except (tf.LookupException, tf.ConnectivityException,tf.ExtrapolationException):
                continue
        if len(trans) > 2 and len(rot) > 2:
            if len(trans) == 4 and len(rot) == 4:
                l1 = trans[0]-trans[2]
                l2 = trans[1]-trans[3]
                mp0 = (trans[0] + trans[1])/2.0
                mp1 = (trans[2] + trans[3])/2.0
                midpoint = (mp1 + mp0)/2.0
            elif len(trans) == 3 and len(rot) == 3:
                l1 = trans[0]-trans[1]
                l2 = trans[0]-trans[2]
            n = np.cross(l1,l2)


if __name__ == '__main__':
    rospy.init_node('mocap_driver')

    publish_tf = rospy.get_param('motion_capture/publish_tf',True)
    my_mocap = mocap_driver(publish_tf)

    rospy.spin()

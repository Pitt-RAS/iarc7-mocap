#!/usr/bin/env python
import rospy
import tf

from image_geometry import PinholeCameraModel
from geometry_msgs.msg import TransformStamped, PointStamped
import cv2

import numpy as np
from math import isnan

class mocap():
    def __init__(self, camera_info, parent_frame, depth_image, rgb_image, cam_model, listener, broadcaster):

        self.center = [0,0]
        #bounding box for Region of interest
        self.roi = [0,0,0,0]

        #bounding box for marker in depth map
        self.marker = [0,0,0,0]

        #ratio of markers height to width (1:1 for circles)
        self.rth = 1.1
        #pixel area of distant marker
        self.th1 = 15
        #pixel area of close marker
        self.th2 = 600

        #create a background subtractor for this object
        self.fgbg = cv2.bgsegm.createBackgroundSubtractorMOG()

        #The frame we want our tf to have as a parent i.e. this is "/map"
        self.parent_frame = parent_frame

        #We need rgb image to locate ROI, and depth_image to get 3D location
        self.depth_image = depth_image
        self.rgb_image = rgb_image

        #The ROI exists in this frame
        self.camera_frame = camera_info.header.frame_id

        #How we figure out 2d/3d projections
        self.cam_model = cam_model

        #Listener is necessary to transform from camera_frame to parent_frame.
        self.listener = listener

        #Broadcaster to publish the transforms.
        self.broadcaster = broadcaster

    #publishes to our view
    def publish(self):
        transform = self._toTransform(self.center[0], self.center[1])
        pos = (transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z)
        rot = (transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w)

        self.broadcaster.sendTransform(pos, rot, rospy.Time.now(), transform.child_frame_id, transform.header.frame_id)

        return transform


    def extract_fg(self,image):
        """
        Takes an RGB image as input. Finds the forground and draws a box around
        it for the ROI.
        """
        #extracts foreground from overall image
        fgmask = fgbg.apply(image)

        #perform opening and closing on image to clean it up
        kernel = np.ones((5,5),np.uint8)
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_CLOSE, kernel)

        #find contours in image
        img, contours, hierarchy = cv2.findContours(fgmask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[0]
        self.roi = cv2.boundingRect(contours)

        img = image
        cv2.rectangle(img,(self.roi[0],self.roi[1]),(self.roi[0]+self.roi[2],self.roi[1]+self.roi[3]),(0,255,0),2)


        cv2.imshow('image',img)
        cv2.imshow('frame',fgmask)


    def label_filter(self,depth_image):
        """
        Takes a depth image as input, finds connected regions and finds marker
        based on how large or small the regions are, and their respective
        bounding box ratios (circles have a bb ratio of 1:1)
        """
        #select roi, we don't want to work on the whole image
        depth_roi_image = self._get_roi(depth_image)

        #clean up noise
        kernel = np.ones((5,5),np.uint8)
        depth_roi_image = cv2.morphologyEx(depth_roi_image, cv2.MORPH_OPEN, kernel)
        depth_roi_image = cv2.morphologyEx(depth_roi_image, cv2.MORPH_CLOSE, kernel)

        #using 4 connectivity here
        connectivity = 4
        output = cv2.connectedComponentsWithStats(depth_roi_image, connectivity, cv2.CV_32S)
        # The first cell is the number of labels
        num_labels = output[0]
        # The second cell is the label matrix
        #labels = output[1]
        # The third cell is the stat matrix
        stats = output[2]
        # The fourth cell is the centroid matrix
        centroids = output[3]
        #extracts ROI from foreground
        #TODO write labelling code from fig 4
        #center of current label set to old
        x,y = [math.inf,math.inf]
        #set default label to -1
        sel_label = -1
        for label in num_labels:
            label_area = stats[label,4]
            if self.th1 < label_area and self.th2 > label_area:
                lx,ly = centroids[label,0],centroids[label,1]
                #is this the lowest label in the list?
                if  [lx,ly] < [x,y]:
                    x,y=lx,ly
                    sel_label = label
                    w,h = stats[label,2], stats[label,3]
                    ratio = max(w/h,h/w)
                    if ratio > self.rth:
                        sel_label = -1
            else:
                sel_label = -1
        if sel_label > -1:
            #update marker's center location w.r.t full image (rect)
            self.center = [centroids[sel_label,0]+self.roi[0],centroids[sel_label,1]+self.roi[1]]
            #update marker location (rect) with new bounding box w.r.t full image
            self.marker = stats[sel_label,0:4]
            self.marker[0],self.marker[1] = self.marker[0] + self.roi[0],self.marker[1] + self.roi[1]
        else:
            #Don't update marker location

##Private methods
##
    #Takes our data and makes a tf2 transform message.
    def _toTransform(self, my_x, my_y):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self.camera_frame
        transform.child_frame_id = "quad_marker"

        (x,y,z) = self._projectTo3d(my_x, my_y)
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z

        transform.transform.rotation.w = 1.0

        #If our parent frame is not the camera frame then an additional transformation is required.
        if self.parent_frame != self.camera_frame:
            point = PointStamped()
            point.header.frame_id = self.camera_frame
            point.header.stamp = rospy.Time(0)
            point.point.x = transform.transform.translation.x
            point.point.y = transform.transform.translation.y
            point.point.z = transform.transform.translation.z

            #Now we've gone from the regular camera frame to the correct parent_frame.
            point_transformed = self.listener.transformPoint(self.parent_frame, point)

            transform.header.frame_id = self.parent_frame
            transform.transform.translation.x = point_transformed.point.x
            transform.transform.translation.y = point_transformed.point.y
            transform.transform.translation.z = point_transformed.point.z

        return transform

    def _projectTo3d(self, x, y):
        [vx,vy,vz] = self.cam_model.projectPixelTo3dRay((x,y))
        _z = self._getDepthAt(x,y)
        _x = vx * _z
        _y = vy * _z
        return (_x, _y, _z)

    def _get_roi(self,image):
        return image[self.roi[0],self.roi[1]),(self.roi[0]+self.roi[2],self.roi[1]+self.roi[3]]

    def _getDepthAt(self, x,y):
        return self.depth_image[y][x]/1000


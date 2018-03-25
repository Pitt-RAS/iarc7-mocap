#!/usr/bin/env python
import rospy
import tf

from image_geometry import PinholeCameraModel
from geometry_msgs.msg import TransformStamped, PointStamped
import cv2

import numpy as np
from math import isnan

class mocap():
    def __init__(self, camera_info, parent_frame, depth_image, rgb_image, cam_model, listener, broadcaster,threshold_ratio,threshold_far,threshold_near):

        self.center
        self.roi = [0,0,0,0]
        self.marker

        #ratio of markers height to width (1:1 for circles)
        self.rth = threshold_ratio
        #pixel area of distant marker
        self.th1 = threshold_far
        #pixel area of close marker
        self.th2 = threshold_near

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



    def extract_fg(self,image):
        #extracts foreground from overall image
        fgmask = fgbg.apply(image)
        cv2.imshow('image',image)
        cv2.imshow('frame',fgmask)


    def label_filter(self,depth_image):
        """
        Takes a depth image as input, finds connected regions and finds marker
        based on how large or small the regions are, and their respective
        bounding box ratios (circles have a bb ratio of 1:1)
        """
        #using 4 connectivity here
        connectivity = 4
        output = cv2.connectedComponentsWithStats(depth_image, connectivity, cv2.CV_32S)
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
        x,y = self.center
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
            #update marker's center location
            self.center = [centroids[sel_label,0],centroids[sel_label,1]]
            #update marker location with new bounding box
            self.marker = stats[sel_label,0:4]
        else:
            #Don't update marker location




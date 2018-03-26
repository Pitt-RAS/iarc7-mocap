#!/usr/bin/env python
import rospy
import tf

import sensor_msgs.point_cloud2 as pc2
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import TransformStamped, PointStamped
import cv2

import numpy as np
import math

class mocap():
    def __init__(self, camera_info, parent_frame, point_cloud2, rgb_image,
            cam_model, listener, broadcaster):

        #bounding box for Region of interest
        self.roi = [100,100,100,100]

        #bounding box for marker in depth map
        self.marker = [0,0,0,0]
        self.center = [0,0]

        #ratio of markers height to width (1:1 for circles)
        self.rth = 1.2
        #pixel area of distant marker
        self.th1 = 15
        #pixel area of close marker
        self.th2 = 10000

        #create a background subtractor for this object
        #self.fgbg = cv2.bgsegm.createBackgroundSubtractorMOG()
        #self.fgbg = cv2.bgsegm.createBackgroundSubtractorCNT()
        self.fgbg = cv2.createBackgroundSubtractorMOG2(1000,20,True)

        #The frame we want our tf to have as a parent i.e. this is "/map"
        self.parent_frame = parent_frame

        #We need rgb image to locate ROI, and pc2 to get 3D location
        self.point_cloud2 = point_cloud2
        self.rgb_image = rgb_image

        #The ROI exists in this frame
        self.camera_frame = camera_info.header.frame_id

        #How we figure out 2d/3d projections
        self.cam_model = cam_model

        #Listener is necessary to transform from camera_frame to parent_frame.
        self.listener = listener

        #Broadcaster to publish the transforms.
        self.broadcaster = broadcaster

    def publish(self,rgb_image,point_cloud2):
        self.point_cloud2 = point_cloud2
        self.rgb_image = rgb_image

        fgmask = self.extract_fg(self.rgb_image)
        self.label_circles(fgmask)
        #self.label_filter(fgmask)
        #if(self.validate()):
        #     self.publish_callback()

    #publishes to our view
    def publish_callback(self):
        transform = self._toTransform(self.center[0], self.center[1])
        pos = (transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z)
        rot = (transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w)

        self.broadcaster.sendTransform(pos, rot, rospy.Time.now(), transform.child_frame_id, transform.header.frame_id)

        print(transform)
        return transform

    def validate(self):
        return (self._validateDepthAt(self.center[0], self.center[1]))

    def extract_fg(self,image):
        """
        Takes an RGB image as input. Finds the forground and draws a box around
        it for the ROI.
        """
        #extracts foreground from overall image
        fgmask = self.fgbg.apply(image)

        #blur and threshold to remove shadows
        fgmask = cv2.GaussianBlur(fgmask,(5,5),0)
        ret1,fgmask = cv2.threshold(fgmask,127,255,cv2.THRESH_BINARY)

        #perform opening and closing on image to clean it up
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(10,10))
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_CLOSE, kernel)
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)

        #find contours in image
        img, contours, hierarchy = cv2.findContours(fgmask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        try: hierarchy = hierarchy[0]
        except: hierarchy = []

        height, width = fgmask.shape
        min_x, min_y = width, height
        max_x = max_y = 0
        for contour, hier in zip(contours, hierarchy):
                (x,y,w,h) = cv2.boundingRect(contour)
                min_x, max_x = min(x, min_x), max(x+w, max_x)
                min_y, max_y = min(y, min_y), max(y+h, max_y)

        img = image.copy()
        cv2.rectangle(img, (min_x,min_y), (min_x+(max_x-min_x),min_y+(max_y-min_y)), (255, 0, 0), 2)
        self.roi = [min_x,min_y,min_x+(max_x-min_x),min_y+(max_y-min_y)]


        #if contours:
        #    c = max(contours, key = cv2.contourArea)
        #    self.roi = cv2.boundingRect(c)

        #cv2.rectangle(img,(self.roi[0],self.roi[1]),(self.roi[0]+self.roi[2],self.roi[1]+self.roi[3]),(0,255,0),2)


        cv2.imshow('image',img)
        cv2.imshow('frame',fgmask)
        k = cv2.waitKey(30) & 0xff

        #return the fgmask
        return fgmask


    def label_circles(self,image):
        """
        Takes a grayscale image as input, finds connected regions and finds marker
        based on how large or small the regions are, and their respective
        bounding box ratios (circles have a bb ratio of 1:1)
        """
        output = self.rgb_image.copy()
        # get contours
        img, contours, hierarchy = cv2.findContours(_get_roi(image), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        contours_area = []
        # calculate area and filter into new array
        for con in contours:
            area = cv2.contourArea(con)
            if self.th1 < area < self.th2:
                contours_area.append(con)

        contours_circles = []
        # check if contour is of circular shape
        for con in contours_area:
            perimeter = cv2.arcLength(con, True)
            area = cv2.contourArea(con)
            if perimeter == 0:
                break
            circularity = 4*math.pi*(area/(perimeter*perimeter))
            print circularity
            if 0.7 < circularity < 1.2:
                contours_circles.append(con)

        #largest circle in ROI is our marker
        marker_circle = max(contours_circles, key = cv2.contourArea)
        #self.roi = cv2.boundingRect(c)

        cv2.drawContours(output,contours_circles,-1,(255,255,0),3)
        # show the output image
        cv2.imshow("output", np.hstack([self.rgb_image, output]))
        k = cv2.waitKey(30) & 0xff

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
        return image[self.roi[1]:(self.roi[1]+self.roi[3]),self.roi[0]:(self.roi[0]+self.roi[2])]

    def _getDepthAt(self, x,y):
        return pc2.read_points(data, field_names=None, skip_nans=False, uvs=[x, y])

    def _validateDepthAt(self, x, y):
        depth = self._getDepthAt(x, y)
        #print depth
        if isnan(depth) or depth == 0:
            return False
        return True


#!/usr/bin/env python
import rospy
import tf

import sensor_msgs.point_cloud2 as pc2
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import TransformStamped, PointStamped
import cv2
import datetime

import numpy as np
import math

class mocap():
    def __init__(self, camera_info, parent_frame, point_cloud2, rgb_image,
            cam_model, listener, broadcaster):

        #bounding box for Region of interest
        self.roi = [100,100,100,100]

        #bounding box for marker in depth map
        self.marker = [0,0,0,0]

        #ratio of markers height to width (1:1 for circles)
        self.rth = 1.2
        #pixel area of distant marker
        self.th1 = 15
        #pixel area of close markermarker_color,
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

        #marker and color pairings
        self.marker_colors = {'front_marker':['RED',[0,0]],'back_marker':['GREEN',[0,0]],'left_marker':['YELLOW',[0,0]],'right_marker':['BLUE',[0,0]]}

        #open logfile
        #logname = "file"
        #self.logfile = open(logname,'w')

    def publish(self,rgb_image,point_cloud2):
        keypress = cv2.waitKey(1) & 0xFF
        if keypress == 32:
            #reset MOG
            self.fgbg = cv2.createBackgroundSubtractorMOG2(1000,20,True)
        elif keypress == ord('q'):
            return 0
            #self.logfile.close()
            #logname = datetime.now().strftime('iarc7_mocap_%H_%M_%d_%m_%Y.log')
            #self.logfile = open(logname,'w')

        self.point_cloud2 = point_cloud2
        self.rgb_image = rgb_image

        fgmask = self.extract_fg(self.rgb_image)
        # Bitwise-AND fgmask and original image
        fgrgb = cv2.bitwise_and(self.rgb_image,self.rgb_image, mask= fgmask)

        #cv2.imshow('frame',fgrgb)
        #k = cv2.waitKey(30) & 0xff

        #key = 'front_marker'
        for key in self.marker_colors:
            #segment image for color
            fgcolor = self.segment_rgb(fgrgb,self.marker_colors[key][0])
            #fgcolor = fgmask
            #label circles in output image
            self.marker_colors[key][1] = self.label_circles(fgcolor)
            #center = self.label_circles(fgcolor)
            if(self.marker_colors[key][1] and self.validate(self.marker_colors[key][1])):
                self.publish_tf(key,self.marker_colors[key][1])

    #publishes to our view
    def publish_tf(self,frame_id,center):
        transform = self._toTransform(center[0], center[1],frame_id)
        pos = (transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z)
        rot = (transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w)

        self.broadcaster.sendTransform(pos, rot, rospy.Time.now(), transform.child_frame_id, transform.header.frame_id)

        print(transform)
        return transform

    def validate(self,center):
        return (self._validateDepthAt(center[0], center[1]))

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
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_CLOSE, kernel)
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_CLOSE, kernel)

        #find contours in image
        #img, contours, hierarchy = cv2.findContours(fgmask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        #try: hierarchy = hierarchy[0]
        #except: hierarchy = []

        #height, width = fgmask.shape
        #min_x, min_y = width, height
        #max_x = max_y = 0
        #for contour, hier in zip(contours, hierarchy):
        #        (x,y,w,h) = cv2.boundingRect(contour)
        #        min_x, max_x = min(x, min_x), max(x+w, max_x)
        #        min_y, max_y = min(y, min_y), max(y+h, max_y)

        #img = image.copy()
        #cv2.rectangle(img, (min_x,min_y), (min_x+(max_x-min_x),min_y+(max_y-min_y)), (255, 0, 0), 5)
        #self.roi = [min_x,min_y,min_x+(max_x-min_x),min_y+(max_y-min_y)]


        #if contours:
        #    c = max(contours, key = cv2.contourArea)
        #    self.roi = cv2.boundingRect(c)

        #cv2.rectangle(img,(self.roi[0],self.roi[1]),(self.roi[0]+self.roi[2],self.roi[1]+self.roi[3]),(0,255,0),2)


        #cv2.imshow('image',img)
        #cv2.imshow('frame',fgmask)
        #k = cv2.waitKey(30) & 0xff

        #return the fgmask
        return fgmask

    def segment_rgb(self,image,color):

        #get rid of very bright and very dark regions
        delta = 3
        lower_gray = np.array([delta, delta,delta])
        upper_gray = np.array([255-delta, 255-delta,255-delta])
        # Threshold the image to get only selected
        mask = cv2.inRange(image, lower_gray, upper_gray)
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image,image, mask= mask)

        #Convert to HSV space
        hsv_img = cv2.cvtColor(res, cv2.COLOR_BGR2HSV)

        #make np arrays for different HSV slices
        if color == "RED":
            # lower red mask (0-10)
            mask0 = cv2.inRange(hsv_img,np.array([0,50,50]),np.array([10,255,255]))
            # upper red mask (170-180)
            mask1 = cv2.inRange(hsv_img,np.array([170,50,50]),np.array([180,255,255]))
            #make composite mask
            mask = mask0 + mask1
        elif color == "BLUE":
            mask = cv2.inRange(hsv_img,np.array([110,50,50]),np.array([130,255,255]))
        elif color == "YELLOW":
            mask = cv2.inRange(hsv_img,np.array([15, 50, 50]), np.array([35, 255, 255]))
        elif color == "GREEN":
            mask = cv2.inRange(hsv_img,np.array([50, 50, 50]), np.array([70, 255, 255]))

        #blur and threshold to remove shadows
        mask = cv2.GaussianBlur(mask,(5,5),0)

        #perform opening and closing on image to clean it up
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(10,10))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        #cv2.imshow("hsv-output", mask)
        return mask

    def label_circles(self,image):
        """
        Takes a grayscale image as input, finds connected regions and finds marker
        based on how large or small the regions are, and their respective
        bounding box ratios (circles have a bb ratio of 1:1)
        """
        output = self.rgb_image.copy()
        #set center to nothing
        center = None
        # get contours
        img, contours, hierarchy = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        try: hierarchy = hierarchy[0]
        except: hierarchy = []

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
            #print circularity
            if 0.7 < circularity < 1.2:
                contours_circles.append(con)

        if contours_circles:
            #largest circle in ROI is our marker
            marker_circle = max(contours_circles, key = cv2.contourArea)
            #self.marker = cv2.boundingRect(marker_circle)
            M = cv2.moments(marker_circle)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            #update center of marker
            center = [cx,cy]
            #self.roi = cv2.boundingRect(c)

            cv2.drawContours(output,marker_circle,-1,(0,0,255),5)
            #cv2.drawContours(output,contours_circles,-1,(0,0,255),5)
        # show the output image
        #cv2.imshow("output", np.hstack([self.rgb_image,output]))
        k = cv2.waitKey(30) & 0xff
        return center

##Private methods
##
    #Takes our data and makes a tf2 transform message.
    def _toTransform(self, my_x, my_y,frame_id):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self.camera_frame
        transform.child_frame_id = frame_id

        (x,y,z) = self._projectTo3d(my_x, my_y)
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z

        quat = tf.transformations.quaternion_from_euler(math.pi/2, -math.pi/2, 0)
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]

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

        #if(self.logfile):
            #logstr = str([transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z])+"\n"
            #self.logfile.write(logstr)

        return transform

    def _projectTo3d(self, x, y):
        [vx,vy,vz] = self.cam_model.projectPixelTo3dRay((x,y))
        point = self._getDepthAt(x,y)
        _z = point[2]
        _y = point[1]
        _x = point[0]
        #_z = self._getDepthAt(x,y)
        #_x = vx * _z
        #_y = vy * _z
        return (_x, _y, _z)

    def _get_roi(self,image):
        return image[self.roi[1]:(self.roi[1]+self.roi[3]),self.roi[0]:(self.roi[0]+self.roi[2])]

    def _getDepthAt(self, x,y):
        return next(pc2.read_points(self.point_cloud2, field_names=None, skip_nans=False, uvs=[[x, y]]))

    def _validateDepthAt(self, x, y):
        depth = self._getDepthAt(x, y)
        #print depth
        if math.isnan(depth[2]) or depth[2] == 0:
            return False
        return True


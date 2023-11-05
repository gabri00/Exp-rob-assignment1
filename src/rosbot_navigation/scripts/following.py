#!/usr/bin/env python

# Python libs
import sys
import time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist , Point
from std_msgs.msg import Bool,Float64,Int32

VERBOSE = False

class image_feature:

    def __init__(self):
        self.ack= False
        self.reached=False
        self.coord_x = 0.0
        self.coord_y = 0.0
        self.my_list = [11, 12, 13, 15]
        self.id =0
        self.id_sub=rospy.Subscriber('/marker_id', Int32, self.id_callback)
        self.sub_coord_centre = rospy.Subscriber('/coord_center', Point, self.centre_coord_callback)
        self.camera_pub = rospy.Publisher('/exp_rob/camera_position_controller/command', Float64, queue_size=10)
        '''Initialize ros publisher, ros subscriber'''
        rospy.init_node('image_feature', anonymous=True)
        # topic where we publish
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("cmd_vel",Twist, queue_size=1)

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/camera/color/image_raw/compressed",CompressedImage, self.callback,  queue_size=1)
        self.ack_detected_sub=rospy.Subscriber("/frame_size_ack",Bool,self.ack_callback,queue_size=1)
        self.ack_reached_sub=rospy.Subscriber("/reached_ack",Bool,self.reached_callback,queue_size=1)
    
    def id_callback(self, msg):
        self.id = msg.data
        
    def centre_coord_callback(self, msg):
        self.coord_x = msg.x
        self.coord_y = msg.y
    
    def ack_callback(self,msg):
        self.ack=msg.data
    
    def reached_callback(self,msg):
        self.reached=msg.data
    
    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE:
            print ('received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

        blackLower = (0, 0, 0)
        blackUpper = (0, 0, 0)

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, blackLower, blackUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found
        if self.ack and self.id == self.my_list[0]:
            
            vel_camera=Float64()
            vel_camera = 0.0 
            self.camera_pub.publish(vel_camera)
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            
            center = (int (self.coord_x), int (self.coord_y))
            
            
            if self.reached:
                
                vel = Twist()
                vel.linear.x = 0.0
                vel.angular.z=0.0
                self.vel_pub.publish(vel)
                self.my_list.pop(0)
                self.reached=False
                
            else:
                cv2.circle(image_np, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                vel = Twist()
                vel.linear.x = -0.001*(radius-200)
                vel.angular.z = 0.002*(center[0]-100)
                self.vel_pub.publish(vel)

        else:
            self.ack=False
            vel_camera=Float64()
            vel_camera = 0.5  
            self.camera_pub.publish(vel_camera)

        cv2.imshow('window', image_np)
        cv2.waitKey(2)

        # self.subscriber.unregister()

def main(args):
    ic = image_feature()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

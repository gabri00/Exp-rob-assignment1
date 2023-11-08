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
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import Twist , Point
from std_msgs.msg import Bool,Float64,Int32
from nav_msgs.msg import Odometry 
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class real_robot_controller:
    def __init__(self):
        rospy.init_node('real_robot_controller', anonymous=True)
    	
        self.detected_ack = False
        self.reached_ack = False
        self.camera_center_x = 0.0
        self.camera_center_y = 0.0
        self.coord_x = 0.0
        self.coord_y = 0.0
        self.my_list = [11, 12, 13, 15]
        self.id = 0
        
        # topic where we publish
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("/cmd_vel",Twist, queue_size=1)

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, self.callback, queue_size=1)
        self.subscriber = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_center_callback, queue_size=1)
        self.ack_detected_sub=rospy.Subscriber("/frame_size_ack", Bool, self.detected_callback, queue_size=1)
        self.ack_reached_sub=rospy.Subscriber("/reached_ack", Bool, self.reached_callback, queue_size=1)
        self.id_sub=rospy.Subscriber('/marker_id', Int32, self.id_callback)
        self.sub_coord_centre = rospy.Subscriber('/coord_center', Point, self.centre_coord_callback)
        
    def camera_center_callback(self, msg):
        self.camera_center_x = (msg.width)/2
        self.camera_center_y = (msg.height)/2
        
    def id_callback(self, msg):
        self.id = msg.data
        
    def centre_coord_callback(self, msg):
        self.coord_x = msg.x
        self.coord_y = msg.y
        
    def detected_callback(self, msg):
        self.detected_ack = msg.data
        
    def reached_callback(self, msg):
        self.reached_ack = msg.data
        
    def callback(self):
        
        marker_center = (int (self.coord_x), int (self.coord_y))
        print("Marker center: ", marker_center)
        
        camera_center = (int (self.camera_center_x), int (self.camera_center_y))
        print("Camera center: ", camera_center)
        
        # only proceed if at least one contour was found
        if self.detected_ack and self.id == self.my_list[0]:
            if self.reached_ack:
                vel = Twist()
                vel.linear.x = 0.0
                vel.angular.z = 0.0
                self.vel_pub.publish(vel)
                self.my_list.pop(0)  
                self.reached_ack = False 
                
            elif (self.coord_x < ((self.camera_center_x) + 15)) and (self.coord_x > ((self.camera_center_x) - 15)):
                vel = Twist()
                vel.angular.z = 0.0
                vel.linear.x = 0.5
                self.vel_pub.publish(vel)
            else:
                vel = Twist()
                vel.linear.x = 0.0
                vel.angular.z = 0.5
                self.vel_pub.publish(vel)
        else:
            vel = Twist()
            vel.linear.x = 0.0
            vel.angular.z = 0.5
            self.vel_pub.publish(vel)

def main():
    real_robot_controller()

    rospy.spin()

if __name__ == '__main__':
    main()

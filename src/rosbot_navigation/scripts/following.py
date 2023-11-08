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
from nav_msgs.msg import Odometry 
from tf.transformations import euler_from_quaternion, quaternion_from_euler

VERBOSE = False

class image_feature:

    def __init__(self):
        rospy.init_node('sim_robot_controller', anonymous=True)
    	
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
        self.camera_pub.= rospy.Publisher("/exp_rob/camera_position_controller/command",Float64, queue_size=1)
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
    def odometry_callback(self, msg):
        global roll, pitch, yaw
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        print(yaw)
        
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
        #np_arr = np.fromstring(ros_data.data, np.uint8)
        #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

        
        marker_center = (int (self.coord_x), int (self.coord_y))
        #print("Marker center: ", marker_center)
        
        camera_center = (int (self.camera_center_x), int (self.camera_center_y))
        #print("Camera center: ", camera_center)
        
        # only proceed if at least one contour was found
        if self.my_list :
            if self.detected_ack and self.id == self.my_list[0]:
                if self.reached_ack and self.id == self.my_list[0]:
                    vel = Twist()
                    vel.linear.x = 0.0
                    vel.angular.z = 0.0
                    self.vel_pub.publish(vel)
                    self.my_list.pop(0) 
                    print("reached: ",self.id)
                    #print(self.my_list) 
                    self.reached_ack = False 
                
                elif (self.camera_center_x < ((self.coord_x ) + 15)) and (self.camera_center_x > (( self.coord_x ) - 15)):
                    vel_camera=Float64()
                    vel_camera = 0.0 
                    self.camera_pub.publish(vel_camera)
                else:
                    vel_camera = Float64()
                    if self.camera_center_x > self.coord_x:
                        vel_camera = 0.2
                    else:
                        vel_camera = -0.2
                    self.camera_pub.publish(vel_camera)
            else:
               self.ack=False
               vel_camera=Float64()
               vel_camera = 0.5  
               self.camera_pub.publish(vel_camera)
        else:
            exit(0)

        

        cv2.imshow('window', image_np)
        cv2.waitKey(2)

        # self.subscriber.unregister()


def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

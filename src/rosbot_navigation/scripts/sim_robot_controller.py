#!/usr/bin/env python

# Python libs
import sys
import time
import math

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
from gazebo_msgs.msg import LinkStates
from tf import transformations

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
        self.err_yaw=0.0
        
        
        # topic where we publish
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("/cmd_vel",Twist, queue_size=1)
        self.camera_pub= rospy.Publisher("/exp_rob/camera_position_controller/command",Float64, queue_size=1)
        # subscribed Topic
        self.subscriber = rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, self.callback, queue_size=1)
        self.subscriber = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_center_callback, queue_size=1)
        self.ack_detected_sub=rospy.Subscriber("/frame_size_ack", Bool, self.detected_callback, queue_size=1)
        self.ack_reached_sub=rospy.Subscriber("/reached_ack", Bool, self.reached_callback, queue_size=1)
        self.id_sub=rospy.Subscriber('/marker_id', Int32, self.id_callback)
        self.sub_coord_centre = rospy.Subscriber('/coord_center', Point, self.centre_coord_callback)
        self.sub_joint_states= rospy.Subscriber('/gazebo/link_states', LinkStates, self.joint_states_callback)
        
    def joint_states_callback(self, msg):
        quaternion_camera_link=(msg.pose[10].orientation.x,
        msg.pose[10].orientation.y,
        msg.pose[10].orientation.z,
        msg.pose[10].orientation.w)
        quaternion_base_link=(msg.pose[9].orientation.x,
        msg.pose[9].orientation.y,
        msg.pose[9].orientation.z,
        msg.pose[9].orientation.w)
		
        euler_cam = transformations.euler_from_quaternion(quaternion_camera_link)
        yaw_camera = euler_cam[2]
        euler_base = transformations.euler_from_quaternion(quaternion_base_link)
        yaw_base = euler_base[2]
        
        angle = (yaw_camera - yaw_base)
        if(math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    
        self.err_yaw= angle
        
            
    def camera_center_callback(self, msg):
        self.camera_center_x = (msg.width)/2
        self.camera_center_y = (msg.height)/2   
    
        
    def id_callback(self, msg):
        self.id = msg.data
        
    def centre_coord_callback(self, msg):
        self.coord_x = msg.x
        self.coord_y = msg.y
    def detected_callback(self,msg):
        self.detected_ack=msg.data
    def reached_callback(self,msg):
        self.reached_ack=msg.data
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
                print("sono dentro qui")
                vel_camera=Float64()
                vel_camera = 0.0 
                self.camera_pub.publish(vel_camera)
                if self.reached_ack and self.id == self.my_list[0]:
                    vel = Twist()
                    vel.linear.x = 0.0
                    vel.angular.z = 0.0
                    self.vel_pub.publish(vel)
                    self.my_list.pop(0) 
                    print("reached: ",self.id)
                    #print(self.my_list) 
                    self.reached_ack = False 
                
                elif (self.camera_center_x < ((self.coord_x ) + 15)) and (self.camera_center_x > (( self.coord_x ) - 15) and math.fabs(self.err_yaw) <= math.pi/90):
                    vel_camera=Float64()
                    vel_camera = 0.0 
                    vel_robot=Twist()
                    vel_robot.linear.x=0.3
                    print("sono dentro elif")
                    vel_robot.angular.z=self.err_yaw
                    self.camera_pub.publish(vel_camera)
                    self.vel_pub.publish(vel_robot)
                else:
                    print("else")
                    vel_cam=Float64()
                    
                    if self.camera_center_x > self.coord_x:
                        vel_cam = 0.2
                    else:
                        vel_cam = -0.2
                    
                    twist_msg = Twist()
                    if math.fabs(self.err_yaw) > math.pi/90:
					
                        twist_msg.angular.z = 3.0*self.err_yaw
                        vel_cam = -3.0*self.err_yaw
                        if twist_msg.angular.z > 0.6:
                            twist_msg.angular.z = 0.6
                            vel_cam = -0.6
                        elif twist_msg.angular.z < -0.5:
                            twist_msg.angular.z = -0.5
                            vel_cam = 0.5
                    self.vel_pub.publish(twist_msg)
                    self.camera_pub.publish(vel_cam)
            else:
               self.detected_ack=False
               vel_camera=Float64()
               vel_camera = 0.5  
               self.camera_pub.publish(vel_camera)
        else:
            exit(0)

        

        

        # self.subscriber.unregister()


def main(args):
    '''Initializes and cleanup ros node'''
    sim_node= image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

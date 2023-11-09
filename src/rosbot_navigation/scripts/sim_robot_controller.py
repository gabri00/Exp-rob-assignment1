#!/usr/bin/env python

# Python libs
import sys
import time
import math

import rospy

from tf import transformations
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Bool, Float64, Int32
from geometry_msgs.msg import Twist , Point
from sensor_msgs.msg import CompressedImage, CameraInfo

class sim_robot_controller:
    def __init__(self):
        rospy.init_node('sim_robot_controller', anonymous=True)
    	
        self.detected_ack = False
        self.reached_ack = False
        self.camera_center = Point()
        self.marker_center = Point()
        self.markers = [11, 12, 13, 15]
        self.marker_id = 0
        self.err_yaw=0.0
        
        # Publisher
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("/cmd_vel",Twist, queue_size=1)
        self.camera_pub= rospy.Publisher("/exp_rob/camera_position_controller/command",Float64, queue_size=1)

        # Subscribers
        rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, self.control_loop, queue_size=1)
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_center_callback, queue_size=1)
        rospy.Subscriber("/ack/detected", Bool, self.detected_callback, queue_size=1)
        rospy.Subscriber("/ack/reached", Bool, self.reached_callback, queue_size=1)
        rospy.Subscriber('/marker/id', Int32, self.marker_id_callback, queue_size=1)
        rospy.Subscriber('/marker/center', Point, self.marker_center_callback, queue_size=1)
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.joint_states_callback, queue_size=1)
        
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
        self.camera_center.x = msg.width / 2
        self.camera_center.y = msg.height / 2 
        
    def marker_id_callback(self, msg):
        self.marker_id = msg.data
        
    def marker_center_callback(self, msg):
        self.marker_center.x = msg.x
        self.marker_center.y = msg.y

    def detected_callback(self,msg):
        self.detected_ack = msg.data

    def reached_callback(self,msg):
        self.reached_ack = msg.data

    def control_loop(self, msg: CompressedImage):        
        
        if self.markers :
            vel_camera = Float64()
            vel_robot = Twist()

            if self.detected_ack and self.marker_id == self.markers[0]:
                vel_camera = 0.0 
                self.camera_pub.publish(vel_camera)

                if self.reached_ack and self.marker_id == self.markers[0]:
                    vel_robot.linear.x = 0.0
                    vel_robot.angular.z = 0.0
                    self.vel_pub.publish(vel_robot)
                    self.markers.pop(0) 
                    self.reached_ack = False 
                    print("Reached: ",self.marker_id)
                
                elif (self.camera_center.x < ((self.marker_center.x) + 12)) and (self.camera_center.x > (( self.marker_center.x) - 12) and math.fabs(self.err_yaw) <= math.pi / 90):
                    vel_camera = 0.0 
                    vel_robot.linear.x = 0.3
                    vel_robot.angular.z = self.err_yaw
                    self.camera_pub.publish(vel_camera)
                    self.vel_pub.publish(vel_robot)
                else:
                    if self.camera_center.x > self.marker_center.x:
                        vel_camera = 0.2
                    else:
                        vel_camera = -0.2
                    
                    if math.fabs(self.err_yaw) > math.pi/90:
                        vel_robot.angular.z = 3.0*self.err_yaw
                        vel_camera = -3.0*self.err_yaw
                        if vel_robot.angular.z > 0.6:
                            vel_robot.angular.z = 0.6
                            vel_camera = -0.6
                        elif vel_robot.angular.z < -0.5:
                            vel_robot.angular.z = -0.5
                            vel_camera = 0.5

                    self.vel_pub.publish(vel_robot)
                    self.camera_pub.publish(vel_camera)
            else:
               self.detected_ack = False
               vel_camera = 0.5 
               self.camera_pub.publish(vel_camera)
        else:
            print("All markers reached!")
            rospy.signal_shutdown("")


def main():
    time.sleep(5)
    sim_robot_controller()
    rospy.spin()

if __name__ == '__main__':
    main()

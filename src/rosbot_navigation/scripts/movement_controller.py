#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

class RobotController:
    def __init__(self):
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub_frame_size_ack = rospy.Subscriber('/frame_size_ack', Bool, self.ack_callback)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.sub_coord_centre = rospy.Subscriber('/coord_center', Point, self.centre_coord_callback)
        self.sub_detected_ack = rospy.Subscriber('/detected_marker', Bool, self.detected_marker_callback)
        
        self.ack_received = False
        self.Kl = 4.0
        self.Ka = 4.0
        self.pos_x=0.0
        self.pos_y=0.0
        self.coord_x=0.0
        self.coord_y=0.0
        self.detected_ack = False
        
    def detected_marker_callback(self, msg):
        self.detected_ack = msg.data
        
    def centre_coord_callback(self, msg):
        self.coord_x = msg.x
        self.coord_y = msg.y
        
    def ack_callback(self, msg):
        if msg.data:
            self.ack_received = True
        
    def odom_callback(self,msg):
            self.pos_x = msg.pose.pose.position.x
            self.pos_y = msg.pose.pose.position.y
    
            # compute distance error
            e_d = math.sqrt((self.pos_x - self.coord_x)**2 + (self.pos_y - self.coord_y)**2)
            # compute angular error
            e_a = math.atan2(self.coord_y - self.pos_y, self.coord_x - self.pos_x) 
            # Clip rotation so it's in [-pi, pi]
            if e_a > math.pi:
                e_a -= 2*math.pi
            elif e_a < -math.pi:
                e_a += 2*math.pi 

            # exit loop if goal reached
            while self.ack_received:
                # Ferma il robot
                stop_cmd_vel_msg = Twist()
                stop_cmd_vel_msg.linear.x = 0.0
                stop_cmd_vel_msg.angular.z = 0.0
                self.pub_cmd_vel.publish(stop_cmd_vel_msg)
           
            # Compute control inputs
            lin_control = self.Kl * e_d 
            ang_control = self.Ka * e_a

            # Build twist msg
            cmd_vel = Twist()
            cmd_vel.linear.x = lin_control
            cmd_vel.angular.z = ang_control

            # Publish it
            self.pub_cmd_vel.publish(cmd_vel)

		
        
        #err_ang = math.atan2((self.coord_y - self.pos_y) ,(self.coord_x - self.pos_x))
        #print(err_ang)
        #if err_ang > self.threshold:
        #    cmd_vel_msg = Twist()
        #    cmd_vel_msg.linear.z = 0.2  # Esempio: imposta la velocità lineare desiderata
        #    self.pub_cmd_vel.publish(cmd_vel_msg)
            
            
			
		 

if __name__ == '__main__':
    rospy.init_node('robot_controller', anonymous=True)
    controller = RobotController()
    
    rospy.spin()


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
        self.sub_odom= rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.sub_coord_centre= rospy.Subscriber('/coord_centre', Point, self.centre_coord_callback)
        self.ack_received = False
        self.threshold=0.7
        self.pos_x=0.0
        self.pos_y=0.0
        self.coord_x=0.0
        self.coord_y=0.0
        
    def centre_coord_callback(self, msg):
        self.coord_x=msg.x
        self.coord_y=msg.y
        
    def ack_callback(self, data):
        if data.data:
            self.ack_received = True

    def move_robot(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown() and not self.ack_received:
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0.2  # Esempio: imposta la velocità lineare desiderata

            self.pub_cmd_vel.publish(cmd_vel_msg)
            rate.sleep()

        # Ferma il robot
        stop_cmd_vel_msg = Twist()
        stop_cmd_vel_msg.linear.x = 0.0
        self.pub_cmd_vel.publish(stop_cmd_vel_msg)
        
    def odom_callback(self,msg):
		
        self.pos_x=msg.pose.pose.position.x
        self.pos_y=msg.pose.pose.position.y
        err_ang = math.atan2((self.coord_y - self.pos_y) ,(self.coord_x - self.pos_x))
        print(err_ang)
        if err_ang > self.threshold:
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.z= 0.2  # Esempio: imposta la velocità lineare desiderata
            self.pub_cmd_vel.publish(cmd_vel_msg)
            
            
			
		 

if __name__ == '__main__':
    rospy.init_node('robot_controller', anonymous=True)
    controller = RobotController()

    controller.move_robot()
    rospy.spin()


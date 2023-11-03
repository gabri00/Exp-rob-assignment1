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
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point  # For ArUco marker center coordinates
from nav_msgs.msg import Odometry

VERBOSE = False

class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        rospy.init_node('image_feature', anonymous=True)
        # topic where we publish the camera image
        self.image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size=1)

        # topic where we publish the robot's velocity commands
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # Subscribe to the camera image topic
        self.subscriber = rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, self.image_callback,  queue_size=1)

        # Subscribe to the ArUco marker center topic (assumed to be /aruco/center)
        self.marker_subscriber = rospy.Subscriber("/coord_center", Point, self.marker_callback,  queue_size=1)

        # Subscribe to the /odom topic
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=1)

        self.marker_center = None  # Variable to store ArUco marker center
        self.robot_position = None  # Variable to store the robot's position

    def image_callback(self, ros_data):
        '''Callback function of subscribed camera image topic.
        Here, images get converted and features detected.'''
        if VERBOSE:
            print ('received image of type: "%s"' % ros_data.format)

        #### Direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

        self.control_robot(image_np)

    def marker_callback(self, marker_data):
        # Callback for ArUco marker center coordinates
        self.marker_center = (marker_data.x, marker_data.y)

    def odom_callback(self, odom_data):
        # Extract and store the robot's position from the Odometry message
        self.robot_position = odom_data.pose.pose.position

    def control_robot(self, image_np):
        if self.marker_center is not None and self.robot_position is not None:
            # Calcola l'errore tra il centro del marker e la posizione del robot
            marker_center_x, marker_center_y = self.marker_center
            robot_x = self.robot_position.x
            robot_y = self.robot_position.y
            error_x = marker_center_x - robot_x
            error_y = marker_center_y - robot_y

            # Implementa un algoritmo di controllo (ad esempio, PID) per regolare il movimento del robot
            k_p_linear = 0.01
            k_p_angular = 0.002
            vel = Twist()
            vel.angular.z = k_p_angular * error_x
            vel.linear.x = k_p_linear * error_y

            # Puoi utilizzare la posizione del robot (self.robot_position) nel tuo algoritmo di controllo
            # Ad esempio, regola la velocità lineare in base alla distanza tra il marker e il robot
            # vel.linear.x *= some_scaling_factor_based_on_distance
            self.vel_pub.publish(vel)
        else:
            # Nessuna informazione sul centro del marker disponibile, arresta il robot o esegui un'azione predefinita
            # vel = Twist()
            # vel.linear.x = 0.5
            # self.vel_pub.publish(vel)
            pass

        # Mostra l'immagine con il marker ArUco e le informazioni di controllo
        cv2.imshow('window', image_np)
        cv2.waitKey(2)

def main(args):
    '''Initializes and cleans up the ROS node'''
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


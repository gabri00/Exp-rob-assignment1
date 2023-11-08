#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Twist , Point
from sensor_msgs.msg import CompressedImage, CameraInfo

class real_robot_controller:
    def __init__(self):
        rospy.init_node('real_robot_controller', anonymous=True)

        self.detected_ack = False
        self.reached_ack = False
        self.camera_center = Point()
        self.marker_center = Point()
        self.markers = [11, 12, 13, 15]
        self.id = 0
        
        # Publishers
        self.image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Subscribers
        rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, self.control_loop, queue_size=1)
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_center_callback, queue_size=1)
        rospy.Subscriber("/ack/detected", Bool, self.detected_callback, queue_size=1)
        rospy.Subscriber("/ack/reached", Bool, self.reached_callback, queue_size=1)
        rospy.Subscriber('/marker/id', Int32, self.marker_id_callback)
        rospy.Subscriber('/marker/center', Point, self.marker_center_callback)

    # Callback to get the center of the camera
    def camera_center_callback(self, msg : CameraInfo):
        self.camera_center.x = msg.width / 2
        self.camera_center.y = msg.height / 2

    # Callback to get the ID of the marker
    def marker_id_callback(self, msg : Int32):
        self.id = msg.data

    # Callback to get the center of the marker
    def marker_center_callback(self, msg : Point):
        self.marker_center.x = msg.x
        self.marker_center.y = msg.y

    # Detection ack callback
    def detected_callback(self, msg : Bool):
        self.detected_ack = msg.data

    # Reached ack callback
    def reached_callback(self, msg : Bool):
        self.reached_ack = msg.data

    # Control loop, runs every time a new image is published
    def control_loop(self, msg : CompressedImage):
        # Print the marker center and the camera center for debugging
        # print(f"Marker center: ({self.marker_center.x}, {self.marker_center.y})")
        # print(f"Camera center: ({self.camera_center.x}, {self.camera_center.y})")
        
        # Proceed only if at least one marker was found
        if self.markers:
            vel = Twist()

            if self.detected_ack and self.id == self.markers[0]:
                if self.reached_ack:
                    vel.linear.x = 0.0
                    vel.angular.z = 0.0
                    self.markers.pop(0)
                    print("Reached: ", self.id)
                    self.reached_ack = False                
                elif (self.camera_center.x < (self.marker_center.x + 10)) and (self.camera_center.x > (self.marker_center.x - 10)):
                    vel.linear.x = 0.5
                    vel.angular.z = 0.0
                else:
                    vel.linear.x = 0.1
                    if self.camera_center.x > self.marker_center.x:
                        vel.angular.z = 0.1
                    else:
                        vel.angular.z = -0.1
            else:
                vel.linear.x = 0.0
                vel.angular.z = 0.5

            self.vel_pub.publish(vel)
        else:
            print("All markers reached!")
            rospy.signal_shutdown("")


def main():
    real_robot_controller()
    rospy.spin()

if __name__ == '__main__':
    main()

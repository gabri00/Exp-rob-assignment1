#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class RobotController:
    def __init__(self):
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub_frame_size_ack = rospy.Subscriber('/frame_size_ack', Bool, self.ack_callback)
        self.ack_received = False

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

if __name__ == '__main__':
    rospy.init_node('robot_controller', anonymous=True)
    controller = RobotController()

    try:
        controller.move_robot()
    except rospy.ROSInterruptException:
        pass


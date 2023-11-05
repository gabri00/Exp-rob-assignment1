#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import Int32

def ack_callback(msg):
    if msg.data:
        vel_msg = Float64()
        vel_msg = 0.3
        vel_pub.publish(vel_msg)
        
def id_callback(msg):
    if (msg.data == markers_list[0]):
        markers_list.pop(0)

        vel_msg = Float64()
        vel_msg = 0.0
        vel_pub.publish(vel_msg)
        
        ack_msg = Bool()
        ack_msg = True
        ack_pub.publish(ack_msg)
    else:
        vel_msg = Float64()
        vel_msg = 0.3
        vel_pub.publish(vel_msg)

def main():
    rospy.init_node('camera_controller')

    global vel_pub, ack_pub, markers_list
    markers_list = [11, 12, 13, 15]

    vel_pub = rospy.Publisher('/exp_rob/camera_position_controller/command', Float64, queue_size=10)
    ack_pub = rospy.Publisher('/detected_maker', Bool, queue_size=10)

    rospy.Subscriber('/frame_size_ack', Bool, ack_callback)
    rospy.Subscriber('/marker_id', Int32, id_callback)

    rospy.spin()

if __name__ == '__main__':
    main()

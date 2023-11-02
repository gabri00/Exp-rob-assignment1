#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import Int32

def ack_callback(msg):
    
    ack=msg.data
    
    if (ack):
    

        
        
        vel_camera=Float64()
    
    
        vel_camera = 0.5  
    
    
        vel_pub.publish(vel_camera)
        
def id_callback(msg):
    
    ack_id=msg.data
    
    if (ack_id==my_list[0]):
    
        my_list.pop(0)
        vel_camera=Float64()
    
    
        vel_camera = 0.0 
    
    
        vel_pub.publish(vel_camera)

def main():
    rospy.init_node('control_node')
    
    # Crea un publisher per la topic cmd_vel.
    global vel_pub
    global my_list
    my_list = [11, 12, 13, 15]

    vel_pub = rospy.Publisher('/exp_rob/camera_position_controller/command', Float64, queue_size=10)
    
    # Crea un subscriber per la topic /ack.
    rospy.Subscriber('/frame_size_ack', Bool, ack_callback)
    
    rospy.Subscriber('/marker_id', Int32, id_callback)
    
    rospy.spin()

if __name__ == '__main__':
    main()

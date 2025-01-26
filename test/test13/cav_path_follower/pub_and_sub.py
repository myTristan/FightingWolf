#!/usr/bin/env python

import numpy as np
import rospy
from cav_msgs.msg import Control, VehicleState
import matplotlib.pyplot as plt
import math
import time
import csv
import trajectory_generate_UTM as tg
import controler as ct


def position_callback(msg):
    print("I got a vehicle state message. its speed:",msg.speed_x, \
        "steering", msg.steer_state, \
        "x", msg.x, "y",  msg.y, \
        "heading", msg.heading )
    
if __name__ == '__main__':
    rospy.init_node('test_node', anonymous=True)
    rospy.Subscriber('vehicle/vehicle_state',VehicleState,position_callback)
    pub = rospy.Publisher('/vehicle/control2bywire', Control, queue_size=10)

    while not rospy.is_shutdown():

        control_msg = Control()
        control_msg.gear_cmd = 4
        control_msg.steering_cmd  = 0.1
        control_msg.speed_cmd = 0
        pub.publish(control_msg)
        print("I published a control message")
        time.sleep(0.02)
#!/usr/bin/env python

import numpy as np
import math
import time
import csv
import trajectory_generate_UTM as tg
import controler as ct


import rospy
from cav_msgs.msg import Control, VehicleState



speed = 1


ini_state = [0,0,0] 
steer_record= []
flag = 0
refer_path = []
start_time =time.time()



import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np

#global vs_msgs
vs_msgs = VehicleState()


def position_callback(msg):
    global vs_msgs
    vs_msgs = msg

    
if __name__ == '__main__':

    rospy.init_node('test_node', anonymous=True)
    rospy.Subscriber('vehicle/vehicle_state',VehicleState,position_callback)
    
    #setup the figure
    f, axes = plt.subplots()
    f.set_figwidth(10) 
    f.set_figheight(10)
    plt.title('Path Follower')
 
    while not rospy.is_shutdown():


        if flag == 0:
            ini_state[0] = vs_msgs.x
            ini_state[1] = vs_msgs.y
            ini_state[2] = vs_msgs.heading
            

            
            # tg.generate_custom_trajectory(refer_path,speed,ini_state,0.04) 


            tg.generate_trajectory(refer_path,speed,ini_state, 0.04)


            # tg.generate_rectangle(refer_path,speed,ini_state, 0.04)

            flag = 1



        running_time = time.time()-start_time

        min_dis = 10000000
        index = 0
        for i in range(len(refer_path)):
            dis = (vs_msgs.x - refer_path[i][0])**2 + (vs_msgs.y - refer_path[i][1])**2
            if dis < min_dis:
                min_dis = dis
                index = i
        print("Index:",index, "Total points", len(refer_path))


        if index + 50 >= len(refer_path):
            rospy.signal_shutdown("Index out of bounds") 

        else:
            steering,theta_diff = ct.pure_pursuit_control(refer_path,index,vs_msgs.x,vs_msgs.y,vs_msgs.heading)


        pub = rospy.Publisher('/vehicle/control2bywire', Control, queue_size=10)
        control_msg = Control()
        control_msg.gear_cmd = 4
        control_msg.steering_cmd  = steering
        control_msg.speed_cmd = speed

        if index >= len(refer_path)-100:
            control_msg.speed_cmd = 0

        pub.publish(control_msg)

        filename='testdata.csv'
        with open(filename,'a',newline='') as csvfile:
            csvwriter = csv.writer(csvfile) 
            csvwriter.writerow([running_time, vs_msgs.x, vs_msgs.y, refer_path[index][0],refer_path[index][1]])  

        print("speed:",speed, "steering_cmd", steering, \
            "x", vs_msgs.x, "y",  vs_msgs.y, \
            "heading", vs_msgs.heading )


        plt.cla()
        axes.set_aspect( 1 )
        plt.xlim(-35 + ini_state[0], 35 + ini_state[0])
        plt.ylim(-35 + ini_state[1], 35 + ini_state[1])


        circle = plt.Circle(( vs_msgs.x , vs_msgs.y ), 1, fill = True)
        axes.add_artist( circle )
        #draw its speed and direction
        x1 = vs_msgs.x+vs_msgs.speed_x*10.0*math.cos(vs_msgs.heading)
        y1 = vs_msgs.y+vs_msgs.speed_x*10.0*math.sin(vs_msgs.heading)
        plt.plot(np.array([vs_msgs.x, x1]), np.array([vs_msgs.y, y1]) )



        xlist = np.array([])
        ylist = np.array([])
        for i in range(len(refer_path)):
            xlist = np.append(xlist, refer_path[i][0])
            ylist = np.append(ylist, refer_path[i][1])

        plt.plot(xlist, ylist)


        plt.show(block=False)
        plt.pause(0.0001)

        time.sleep(0.02)

import numpy as np
import math
def pure_pursuit_control(refer_path,index,x,y,heading):

        target_point = refer_path[index+50]
        distance = np.sqrt((target_point[1] - refer_path[index][1])**2 + (target_point[0] - refer_path[index][0])**2)
        y1 = target_point[1] 
        x1 = target_point[0] 

        k_target =(y1-y)/(x1-x)
        if x1-x>=0 and y1-y>=0:
            target_direct = np.arctan(k_target)
        elif x1-x>=0 and y1-y<=0:
            target_direct = 2*np.pi+np.arctan(k_target)
        elif x1-x<=0 and y1-y<=0:
            target_direct =np.pi+np.arctan(k_target)
        else:
            target_direct = np.pi+np.arctan(k_target)

        theta_diff = target_direct-heading

        if theta_diff>np.pi:
            theta_diff = theta_diff-2*np.pi
        elif theta_diff<-np.pi:
            theta_diff = theta_diff+2*np.pi
        steering = math.atan(2*0.7*np.sin(theta_diff)/distance)
        return steering,theta_diff

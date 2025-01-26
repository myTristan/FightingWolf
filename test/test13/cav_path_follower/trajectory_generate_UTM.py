import numpy as np
import math

##################################################
############           ###############
##################################################

AMPLITUDE = 0.5       
WAVELENGTH = 10    


REC_LENGTH = 30     
REC_WIDTH = 15      
RADIUS = 8          


SMALL_RADIUS = 4    
LARGE_RADIUS = 8    
LINE_LENGTH = 2     


##################################################
############           #############
##################################################
def generate_trajectory(refer_path, speed, INIT, dt):
    k = 2 * np.pi / WAVELENGTH
    omega = speed * k

    total_time = 10*WAVELENGTH / speed 

    t = np.arange(0, total_time, dt)

    x = speed * t
    y = AMPLITUDE * ( np.sin(k * x + np.pi / 2) - 1)
    for i in range(len(t)):
        phi = math.atan(y[i] / x[i])
        if x[i] >= 0 and y[i] >= 0:
            phi = phi
        elif x[i] >= 0 and y[i] <= 0:
            phi = 2 * np.pi + phi
        elif x[i] <= 0 and y[i] <= 0:
            phi = np.pi + phi
        else:
            phi = np.pi + phi
        r = math.sqrt(x[i] ** 2 + y[i] ** 2)
        refer_path.append((r * math.cos(phi + INIT[2]) + INIT[0], r * math.sin(phi + INIT[2]) + INIT[1]))


##################################################
############               #############
##################################################   
def generate_rectangle(refer_path, speed, INIT, dt):

    total_time = (2 * REC_LENGTH + 2 * REC_WIDTH + 3 * math.pi * RADIUS / 2) / speed

    t = np.arange(0, total_time, dt)

    x = []
    y = []
    for i in range(len(t)):
        if t[i] < (REC_LENGTH) / speed:
            x.append(speed * t[i])
            y.append(0)
        elif t[i] < (REC_LENGTH + math.pi * RADIUS / 2) / speed:
            theta = speed * (t[i] - REC_LENGTH / speed) / RADIUS
            x.append(REC_LENGTH + RADIUS * math.sin(theta))
            y.append(RADIUS - RADIUS * math.cos(theta))
        elif t[i] < (REC_LENGTH + math.pi * RADIUS / 2 + REC_WIDTH) / speed:
            x.append(REC_LENGTH + RADIUS)
            y.append(speed * (t[i] - (REC_LENGTH + math.pi * RADIUS / 2) / speed) + RADIUS)
        elif t[i] < (REC_LENGTH + math.pi * RADIUS + REC_WIDTH) / speed:
            theta = speed * (t[i] - (REC_LENGTH + REC_WIDTH + math.pi * RADIUS / 2) / speed) / RADIUS
            x.append(REC_LENGTH + RADIUS * math.cos(theta))
            y.append(REC_WIDTH + RADIUS + RADIUS * math.sin(theta))
        elif t[i] < (2 * REC_LENGTH + math.pi * RADIUS + REC_WIDTH) / speed:
            x.append(REC_LENGTH - speed * (t[i] - (REC_LENGTH + math.pi * RADIUS + REC_WIDTH) / speed))
            y.append(REC_WIDTH + 2 * RADIUS)

        elif t[i] < (2 * REC_LENGTH + REC_WIDTH + math.pi * RADIUS) / speed:
            theta = speed * (t[i] - (2 * REC_LENGTH + REC_WIDTH - 2 * RADIUS) / speed) / RADIUS
            x.append(RADIUS - RADIUS * math.cos(theta))
            y.append(REC_WIDTH - RADIUS - RADIUS * math.sin(theta))
        elif t[i] < (2 * REC_LENGTH + REC_WIDTH + 3 * math.pi * RADIUS / 2) / speed:
            theta = speed * (t[i] - (2 * REC_LENGTH + REC_WIDTH + math.pi * RADIUS) / speed) / RADIUS
            x.append(0 - RADIUS * math.sin(theta))
            y.append(REC_WIDTH + RADIUS + RADIUS * math.cos(theta))

        else:
            x.append(-RADIUS)
            y.append(
                REC_WIDTH + RADIUS - speed * (t[i] - (2 * REC_LENGTH + REC_WIDTH + 3 * math.pi * RADIUS / 2) / speed))

    for i in range(len(x)):
        phi = math.atan(y[i] / x[i])
        if x[i] >= 0 and y[i] >= 0:
            phi = phi
        elif x[i] >= 0 and y[i] <= 0:
            phi = 2 * np.pi + phi
        elif x[i] <= 0 and y[i] <= 0:
            phi = np.pi + phi
        else:
            phi = np.pi + phi
        r = math.sqrt(x[i] ** 2 + y[i] ** 2)
        refer_path.append((r * math.cos(phi + INIT[2]) + INIT[0], r * math.sin(phi + INIT[2]) + INIT[1]))


##################################################
############         #############
##################################################
def generate_custom_trajectory(refer_path, speed,INIT, dt):
    total_time = (3 * math.pi * SMALL_RADIUS + math.pi * LARGE_RADIUS / 2 + 2 * LINE_LENGTH) / speed
    t = np.arange(0, total_time, dt)
    targetx=[]
    targety=[]
    for i in range(len(t)):
        if t[i] < LINE_LENGTH / speed:
            x = speed * t[i]
            y = 0
            targetx.append(x)
            targety.append(y)

        elif t[i] >= LINE_LENGTH / speed and t[i] < math.pi * SMALL_RADIUS / (2 * speed) + LINE_LENGTH / speed:
            theta = speed * (t[i] - LINE_LENGTH / speed) / SMALL_RADIUS
            x = LINE_LENGTH + SMALL_RADIUS * math.sin(theta)
            y = SMALL_RADIUS - SMALL_RADIUS * math.cos(theta)
            targetx.append(x)
            targety.append(y)


        elif t[i] >= math.pi * SMALL_RADIUS / 2 / speed + LINE_LENGTH / speed and t[i] < math.pi * (
                LARGE_RADIUS + SMALL_RADIUS) / 2 / speed + LINE_LENGTH / speed:
            theta = speed * (t[i] - (math.pi * SMALL_RADIUS / 2 / speed + LINE_LENGTH / speed)) / LARGE_RADIUS
            x = LINE_LENGTH + SMALL_RADIUS + LARGE_RADIUS * math.cos(theta) - LARGE_RADIUS
            y = SMALL_RADIUS + LARGE_RADIUS * math.sin(theta)
            targetx.append(x)
            targety.append(y)
           

        elif t[i] >= math.pi * (LARGE_RADIUS + SMALL_RADIUS) / 2 / speed + LINE_LENGTH / speed and t[i] < math.pi * (
                LARGE_RADIUS + 3 * SMALL_RADIUS) / 2 / speed + LINE_LENGTH / speed:
            theta = speed * (t[i] - (
                        math.pi * (LARGE_RADIUS + SMALL_RADIUS) / 2 / speed + LINE_LENGTH / speed)) / SMALL_RADIUS
            x = LINE_LENGTH + SMALL_RADIUS - LARGE_RADIUS - SMALL_RADIUS * math.sin(theta)
            y = LARGE_RADIUS + SMALL_RADIUS * math.cos(theta)
            targetx.append(x)
            targety.append(y)
            
        elif t[i] >= math.pi * (LARGE_RADIUS + 3 * SMALL_RADIUS) / 2 / speed + LINE_LENGTH / speed and t[
            i] < math.pi * (LARGE_RADIUS + 3 * SMALL_RADIUS) / 2 / speed + 2 * LINE_LENGTH / speed:
            x = LINE_LENGTH - LARGE_RADIUS + SMALL_RADIUS + speed * (
                        t[i] - math.pi * (LARGE_RADIUS + 3 * SMALL_RADIUS) / 2 / speed - LINE_LENGTH / speed)
            y = LARGE_RADIUS - SMALL_RADIUS
            targetx.append(x)
            targety.append(y)
          
    for i in range (len(targetx)):
            phi = math.atan(targety[i] / targetx[i])
            if targetx[i] >= 0 and targety[i] >= 0:
                phi = phi
            elif targetx[i] >= 0 and targety[i] <= 0:
                phi = 2 * np.pi + phi
            elif targetx[i] <= 0 and targety[i] <= 0: 
                phi = np.pi + phi
            else:
                phi = np.pi + phi
            r = math.sqrt(targetx[i] ** 2 + targety[i] ** 2)

            refer_path.append((r * math.cos(phi + INIT[2]) + INIT[0], r * math.sin(phi + INIT[2]) + INIT[1]))

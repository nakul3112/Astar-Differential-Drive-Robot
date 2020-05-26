#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Apr  2 03:27:10 2019
@author: nakul
"""
# -*- coding: utf-8 -*-
"""
Created on Sat Mar 30 13:03:01 2019
@author: nakul
"""

import numpy as np
import matplotlib.pyplot as plt
import heapq
import math
import time
import rospy
from geometry_msgs.msg import Twist

def ros_func(input_var):
    rospy.init_node('Pathturtlebot', anonymous=True)
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    vel_bot = Twist()
    vel_bot.linear.x = 0.0
    vel_bot.angular.z = 0.0
    rate = rospy.Rate(10)    # Sampling rate
    c = 0
    n = 0
    while not rospy.is_shutdown():
        if (c <= 10):
            if (n == len(input_var)):
                vel_bot.linear.x = 0.0
                vel_bot.angular.z = 0.0
                pub_vel.publish(vel_bot)
                break
        
            x = float("{0:4f}".format(input_var[n][2]/200))
            w = float("{0:4f}".format(input_var[n][1]))
            vel_bot.linear.x = x
	    print("x: ", x)
            vel_bot.angular.z = w
	    print("w: ", w)   
            pub_vel.publish(vel_bot)
            if(c == 10):
                #print(n, vel_bot.linear.x, vel_bot.angular.z)
                n = n + 1
                c = 0
        c += 1
        rate.sleep()
## CODE FOR OBSTACLE MAPPING =================================================
# For boundary
def check_boundary(x, y, r):
    bool1= (x >= 0 and x <= r ) or ( x >= 1110-r and x <= 1110 ) 
    bool2= (y >= 0 and y <= r ) or ( y >= 1010-r and y <= 1010 ) 
    req = False    
    if (bool1 or bool2):
        req = True
    return req
# For rectangles
def check_rect(x, y, r):
    #rect1
    f1 = x - 918 - r
    f2 = 832 - x - r
    f3 = 827 - y  - r 
    f4 = y - 1070 - r
    rec1 = (f1 <= 0 and f2 <= 0 and f3 <=0 and f4 <= 0)
    
    #rect2
    f1_2 = x - 1026 - r
    f2_2 = 983 - x - r
    f3_2 = 919 - y - r
    f4_2 = y - 1010 - r 
    rec2 = (f1_2 <= 0 and f2_2 <= 0 and f3_2 <=0 and f4_2 <= 0)
    
    #rect3
    f1_3 = x - 1110 - r
    f2_3 = 744 - x - r
    f3_3 = 621 - y - r
    f4_3 = y - 697 - r
    rec3 = (f1_3 <= 0 and f2_3 <= 0 and f3_3 <=0 and f4_3 <= 0)
    
    #rect4
    f1_4 = x - 1110 - r
    f2_4 = 1052 - x - r
    f3_4 = 448.5 - y - r
    f4_4 = y - 565.5 - r
    rec4 = (f1_4 <= 0 and f2_4 <= 0 and f3_4 <=0 and f4_4 <= 0)
    
    #rect5
    f1_5 = x - 1110 - r
    f2_5 = 1019 - x - r
    f3_5 = 362.5 - y - r
    f4_5 = y - 448.5 - r
    rec5 = (f1_5 <= 0 and f2_5 <= 0 and f3_5 <=0 and f4_5 <= 0)
    
    #rect6
    f1_6 = x - 1110 - r
    f2_6 = 1052 - x - r
    f3_6 = 178.25 - y - r
    f4_6 = y - 295.25 - r
    rec6 = (f1_6 <= 0 and f2_6 <= 0 and f3_6 <=0 and f4_6 <= 0)
    
    #rect7
    f1_7 = x - 1110 - r
    f2_7 = 927 - x - r
    f3_7 = 35 - y - r
    f4_7 = y - 111 - r
    rec7 = (f1_7 <= 0 and f2_7 <= 0 and f3_7 <=0 and f4_7 <= 0)
    
    #rect8 
    f1_8 = x - 1110 - r 
    f2_8 = 685 - x - r
    f3_8 = 0 - y - r
    f4_8 = y - 35 - r
    rec8 = (f1_8 <= 0 and f2_8 <= 0 and f3_8 <=0 and f4_8 <= 0)
    
    #rect9   
    f1_9 = x - 896 - r
    f2_9 = 779 - x - r
    f3_9 = 35 - y - r
    f4_9 = y - 93 - r
    rec9 = (f1_9 <= 0 and f2_9 <= 0 and f3_9 <=0 and f4_9 <= 0)
    
    #rect10
    f1_10 = x - 748 - r
    f2_10 = 474 - x - r
    f3_10 = 35 - y - r
    f4_10 = y - 187 - r
    rec10 = (f1_10 <= 0 and f2_10 <= 0 and f3_10 <=0 and f4_10 <= 0)
    
    #rect11
    f1_11 = x - 712 - r
    f2_11 = 529 - x - r
    f3_11 = 265 - y - r
    f4_11 = y - 341 - r
    rec11 = (f1_11 <= 0 and f2_11 <= 0 and f3_11 <=0 and f4_11 <= 0)
    
    #rect12
    f1_12 = x - 529 - r 
    f2_12 = 438 - x - r
    f3_12 = 315 - y - r
    f4_12 = y - 498 - r
    rec12 = (f1_12 <= 0 and f2_12 <= 0 and f3_12 <=0 and f4_12 <= 0)
    
    #rect13 
    f1_13 = x - 936.5 - r
    f2_13 = 784.5 - x - r
    f3_13 = 267 - y - r
    f4_13 = y - 384 - r
    rec13 = (f1_13 <= 0 and f2_13 <= 0 and f3_13 <=0 and f4_13 <= 0)
    
    req= False 
    if (rec1 or rec2 or rec3 or rec4 or rec5 or rec6 or rec7 or rec8 or rec9 
        or rec10 or rec11 or rec12 or rec13):
        req = True
    return req  
# For circles
def check_circle(x, y, r):
    eqn_circle_1= (x - 390)**2 + (y - 965)**2 - (40.5 + r)**2
    eqn_circle_2= (x - 438)**2 + (y - 736)**2 - (40.5 + r)**2
    eqn_circle_3= (x - 390)**2 + (y - 45)**2 - (40.5 + r)**2
    eqn_circle_4= (x - 438)**2 + (y - 274)**2 - (40.5 + r)**2    
    req = False
    # using semi-algabraic equation to define obstacle space
    if (eqn_circle_1 <= 0 or eqn_circle_2 <= 0 or eqn_circle_3 <= 0 or eqn_circle_4 <= 0):
        req = True
    return req
# For ellipse
def check_ellipse(x, y, r):
    sq_1 = x - 310 
    sq_2 = 150 - x 
    sq_3 = 750 - y - r
    sq_4 = y - 910 - r
    bool1 = (sq_1 <= 0 and sq_2 <= 0 and sq_3 <= 0 and sq_4 <= 0)
    #r1
    eq_circle_1 = (x - 150)**2 + (y - 830)**2 - (80 + r)**2
    #r2     
    eq_circle_2 = (x - 310)**2 + (y - 830)**2 - (80 + r)**2
    req = False
    if (bool1 or eq_circle_1 <=0 or eq_circle_2 <=0):
        req = True
    return req
#==============================================================================
# ox=[]
# oy=[]
# obstacle = np.zeros(shape=(1111,1011))   ##########
# for i in range(0,1111):
#     for j in range(0,1011):
#         req0 = check_boundary(i, j)
#         req1 = check_rect(i, j)
#         req2 = check_circle(i, j)
#         req3 = check_ellipse(i, j)
#              
#         if (req0 or req1 or req2 or req3):
#             obstacle[i][j]=1
#             ox.append(i)
#             oy.append(j)
#             
# plt.plot(ox,oy,"ko")
# #plt.grid(True)
# plt.axis("equal")
# plt.show()
# plt.ylim((-10, 1200))
# plt.xlim((-10, 1200))  
#==============================================================================

#==============================================================================
            
flag_for_display = True

sx = int(input("Enter x coordinate of start point:"))
sy = int(input("Enter y coordinate of start point:"))
gx = int(input("Enter x coordinate of goal point:"))
gy = int(input("Enter y coordinate of goal point:"))
r = 30
#left_vel = int(input("Enter the left wheel velocity: "))
#right_vel = int(input("Enter the right wheel velocity: "))

startp = (sx, sy)
goalp = (gx, gy)
print("Start:", startp)
print("Goal:", goalp)

ox=[]
oy=[]
obstacle = np.zeros(shape=(1111,1011))   ##########
for i in range(0,1111):
    for j in range(0,1011):
        req0 = check_boundary(i, j, r)
        req1 = check_rect(i, j, r)
        req2 = check_circle(i, j, r)
        req3 = check_ellipse(i, j, r)
             
        if (req0 or req1 or req2 or req3):
            obstacle[i][j]=1
            ox.append(i)
            oy.append(j)
            
plt.plot(ox,oy,"ko")
plt.plot(sx,sy,"go")
plt.plot(gx,gy,"ro")
#plt.grid(True)
plt.axis("equal")
plt.show()
plt.ylim((-10, 1200))
plt.xlim((-10 ,1200))  



start = time.time()
################################################################   

visit_x,visit_y = [],[]

backtrack = []

########################### @@@@@@@@@ ####################
def motion_model(RPM1, RPM2, dt, theta):
    # L is the distance between two wheels of the robot
    L = 35.4
    # r is the radius of the robot wheel
    r = 3.8
    action_space = []

    steps = [[0, RPM1],[RPM1, 0], [RPM1, RPM1],[0, RPM2],
             [RPM2, 0],[RPM2, RPM2],[RPM1, RPM2],[RPM2, RPM1]]
               
    for moves in steps:
        ur = moves[0]
        ul = moves[1]
        thetadot = (r / L) * (ur - ul)
        dtheta = thetadot * dt
        
        change_theta = theta + dtheta                                            # Change
        xdot = (r / 2) * (ul + ur) * math.cos(change_theta)                      # Change
        ydot = (r / 2) * (ul + ur) * math.sin(change_theta)                      # Change
        dx = round(xdot * dt)
        dy = round(ydot * dt)
                                                          
        cost = float(math.sqrt((dx) ** 2 + (dy) ** 2))
        vel_mag = np.sqrt(xdot**2 + ydot**2)                                     # Change
        action_space.append((dx, dy, cost, change_theta, dtheta, vel_mag))             # Change
        
    return action_space
    
########################### @@@@@@@@@ ####################


def astar(sx, sy, gx, gy, ox, oy):    
    
    unvisited_list = []
    
    visited_list = []
    
    theta = 0                                                                          # Change
    start_node = (0,(sx, sy),None,0, theta, 0,0)                                       # Change
    goal_node = (0,(gx, gy),None,0, 0, 0, 0)                                           # Change
    
    #motion = motion_model(5,10,0, 0.5)
    
    heapq.heappush(unvisited_list,(start_node))
    
    obstacle[start_node[1][0]][start_node[1][1]] = 1

    while len(unvisited_list)>0:
        current_node = heapq.heappop(unvisited_list)
        
        heapq.heappush(visited_list,current_node)
        
        motion = motion_model(5,10,0.5,current_node[4])                               # Change
        
        visit_x.append(current_node[1][0])
        visit_y.append(current_node[1][1])
        
        curr_vel = current_node[6]                                                        # Change
        curr_change_angle = current_node[5]                                               # Change

        backtrack.append((current_node[1],current_node[2], curr_change_angle, curr_vel))  # Change
        
        if (len(visit_x) % 1000 == 0 or len(visit_x) == 1) :
            if flag_for_display:
                    plt.plot(visit_x, visit_y, "3g")          
                    plt.pause(0.0001)
        
        
        threshold = round(np.sqrt((current_node[1][0] - goal_node[1][0]) ** 2 + (current_node[1][1] - goal_node[1][1]) ** 2))

        if threshold <= 15:
            path = []
            l = len(backtrack)
            current_pos = backtrack[l-1][0]
            current_vel = backtrack[l-1][3]                                                # Change
            current_change_angle = backtrack[l-1][2]                                       # Change
            path.append((current_pos,current_change_angle,current_vel))                    # Change
            parent = backtrack[l-1][1]
            print("Goal node found!!!")
            
            while parent != None: 
                for i in range(l):
                    X = backtrack[i]
                    if X[0] == parent:
                        parent = X[1]
                        current_pos = X[0]
                        path.append((current_pos,X[2], X[3]))                              # Change
            return path[::-1]

            
        # List of explored neighbors                                                                     
        sub = []         
        for new_pos in motion:
            
            # Get node position
            node_pos = (current_node[1][0] + new_pos[0], current_node[1][1] + new_pos[1])
            
            node_gcost = current_node[3] + new_pos[2]
            
            node_hcost = (math.sqrt((goal_node[1][0] - node_pos[0] )**2 + (goal_node[1][1] - node_pos[1] )**2))
            
            node_hcost = node_gcost + node_hcost
            
            node_parent = current_node[1]
            
            node_change_angle = new_pos[3]                                             # Change
            
            node_vel_pub = new_pos[5]                                                  # Change
            
            node_dtheta = new_pos[4]                                                   # Change
            # Check the boundaries
            if node_pos[0] > (len(obstacle) - 1) or node_pos[0] < 0 or node_pos[1] > (len(obstacle[0]) -1) or node_pos[1] < 0:
                continue
            # Check the obstacles and avoid them
            if obstacle[node_pos[0]][node_pos[1]] != 0:
                continue
            # Create the cost_map
            obstacle[node_pos[0]][node_pos[1]] = 1
            
            # Plotting the nodes
            #plt.plot([current_node[1][0], node_position[0]],[current_node[1][1], node_position[1]])
            
            # Creating a new node and also assigning a parent
            new_node = (node_hcost,node_pos,node_parent,node_gcost, node_change_angle,node_dtheta, node_vel_pub)  # Change              
            sub.append(new_node)
            heapq.heappush(unvisited_list,(new_node))       
            
####################################################################  
    
if startp in zip(ox, oy) or goalp in zip(ox, oy):
    if startp in zip(ox, oy) :
        print("\nStart point is in obstacle space! Please enter valid point.")
    if goalp in zip(ox, oy) :
        print("\nGoal point is in obstacle space! Please enter valid point.")  
else:
    print("\nExploring the nodes...")
    path = astar(sx, sy, gx, gy, ox, oy)
    if len(path) == 0:
        print("\nThe path cannot be obtained because the start point or goal point is outside the map or inside closed obstacle region.")
    else:
        if flag_for_display:
            path_x,path_y = [],[]
            
            for i in range(len(path)):
                path_x.append(path[i][0][0])                                       # Change
                path_y.append(path[i][0][1])                                       # Change
            print('path',path)
            print('path1',path[1][2])
            
            plt.plot(path_x,path_y,"-r",linewidth=2)
            plt.plot(visit_x, visit_y, "3g") 
            plt.show()
#==============================================================================
#             print(path)
#==============================================================================
        #print("\n Explored path: ", req, req1)

end = time.time()
print("\nTime elapsed: ", abs(end-start), "sec \n")

ros_func(path)


















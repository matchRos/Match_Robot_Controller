#! /usr/bin/env python3

from cmath import sqrt
import rospy
from nav_msgs.msg import Path
from mypath import Mypath
from tf import transformations
import matplotlib.pyplot as plt
import math
from scipy.signal import savgol_filter


    

def compute_trajectories(robot0_path,robot1_path,robot2_path,robot0_v,robot0_w,robot1_v,robot1_w,robot2_v,robot2_w):
    control_rate = 100.0
    target_reached = False
    target_pose = Mypath()
    target_path = Mypath()
    target_path.x = []
    target_path.y = []
    target_pose.x = robot0_path.x[0]
    target_pose.y = robot0_path.y[0]
    target_pose.phi = robot0_path.phi[0]
    path_distance = robot0_v[0]
    current_angle = robot0_path.phi[0]
    index = 1

    target_vel_lin = 0.2
    w_limit = 0.2
    
    current_velocity_lin = 0.0
    current_velocity_ang = 0.0
    dist = 0.0
    acc_limit_lin = 0.01
    acc_limit_ang = 0.01

    dist_list = []
    act_dist_list = []
    target_angle_old = 0.0

    #######


    while not rospy.is_shutdown() and target_reached == False:
        acc_lin = target_vel_lin/control_rate - current_velocity_lin

        # limit acceleration
        if abs(acc_lin) > acc_limit_lin:
            acc_lin = acc_lin / abs(acc_lin) * acc_limit_lin

        current_velocity_lin += acc_lin/control_rate

        if dist + current_velocity_lin >= path_distance:
            while dist + current_velocity_lin >= path_distance and not rospy.is_shutdown():
                index += 1
                if index > len(robot0_v)-2:
                    target_reached = True
                    print("target reached")
                    break
                else:
                    path_distance += robot0_v[index]


        dist_to_cp = math.sqrt((robot0_path.y[index]-target_pose.y)**2 + (robot0_path.x[index]-target_pose.x)**2)
        if dist_to_cp<current_velocity_lin:
            index += 1
            print("slow")
        if index >= len(robot0_path.y):
            break

        
        target_angle = math.atan2(robot0_path.y[index]-target_pose.y, robot0_path.x[index]-target_pose.x)
        if abs(target_angle-target_angle_old)>1.0:
            dist_to_cp = math.sqrt((robot0_path.y[index]-target_pose.y)**2 + (robot0_path.x[index]-target_pose.x)**2)
            print(path_distance,dist,dist_to_cp,current_velocity_lin,target_angle)
        target_angle_old = target_angle

        w_target=target_angle-current_angle
        if abs(w_target) > w_limit:
            w_target = w_target / abs(w_target) * w_limit
        current_angle += w_target

        target_pose.x = target_pose.x + math.cos(target_angle) * current_velocity_lin
        target_pose.y = target_pose.y + math.sin(target_angle) * current_velocity_lin
            
        dist += current_velocity_lin / control_rate
        target_path.x.append(target_pose.x)
        target_path.y.append(target_pose.y)
        dist_list.append(path_distance)
        act_dist_list.append(dist)

    xhat = savgol_filter(target_path.x, 51, 3) # window size 51, polynomial order 3
    yhat = savgol_filter(target_path.y, 51, 3) # window size 51, polynomial order 3

    plt.figure()
    f, (ax1, ax2) = plt.subplots(1, 2, sharey=True)
    ax1.plot(xhat,yhat)
    #ax1.set_title('Sharing Y axis')
    #ax1.plot(target_path.x,target_path.y)
    ax2.plot(robot0_path.x,robot0_path.y)
    plt.show()
  

    v_path = [0.0]
    phi_path = [robot0_path.phi[0]]
    phi_path[0] = math.atan2(yhat[1]-yhat[0], xhat[1]-xhat[0])
    w_path = [0.0]

    for i in range(1,len(xhat)):
        v_path.append(math.sqrt((xhat[i]-xhat[i-1])**2+(yhat[i]-yhat[i-1])**2))
        phi_path.append(math.atan2(yhat[i]-yhat[i-1], xhat[i]-xhat[i-1]))
        #v_path.append(math.sqrt((target_path.x[i]-target_path.x[i-1])**2+(target_path.y[i]-target_path.y[i-1])**2))
        #phi_path.append(math.atan2(target_path.y[i]-target_path.y[i-1], target_path.x[i]-target_path.x[i-1]))
        w_path.append(phi_path[i]-phi_path[i-1])
    plt.figure()
    plt.plot(v_path)
    plt.plot(w_path)
    plt.show()





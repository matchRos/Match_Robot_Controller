#! /usr/bin/env python3

from cmath import sqrt
import rospy
#from nav_msgs.msg import Path
from mypath import Mypath
#from tf import transformations
import matplotlib.pyplot as plt
import math
from scipy.signal import savgol_filter


    

def compute_trajectories(robot0_path,robot1_path,robot2_path,robot0_v,robot0_w,robot1_v,robot1_w,robot2_v,robot2_w):
    control_rate = 100.0
    target_reached = False
    robot0_target_pose = Mypath()
    robot0_target_path = Mypath()
    robot0_target_path.x = []
    robot0_target_path.y = []
    robot0_target_pose.x = robot0_path.x[0]
    robot0_target_pose.y = robot0_path.y[0]
    robot0_target_pose.phi = robot0_path.phi[0]
    robot1_target_pose = Mypath()
    robot1_target_path = Mypath()
    robot1_target_path.x = []
    robot1_target_path.y = []
    robot1_target_pose.x = robot1_path.x[0]
    robot1_target_pose.y = robot1_path.y[0]
    robot1_target_pose.phi = robot1_path.phi[0]
    robot2_target_pose = Mypath()
    robot2_target_path = Mypath()
    robot2_target_path.x = []
    robot2_target_path.y = []
    robot2_target_pose.x = robot2_path.x[0]
    robot2_target_pose.y = robot2_path.y[0]
    robot2_target_pose.phi = robot2_path.phi[0]
    robot0_path_distance = robot0_v[0]
    robot0_current_angle = robot0_path.phi[0]
    robot0_current_angle = robot1_path.phi[0]
    robot0_current_angle = robot2_path.phi[0]
    index = 1

    target_vel_lin = 0.2
    w_limit = 0.2
    
    robot0_current_velocity_lin = 0.0
    robot1_current_velocity_lin = 0.0
    robot2_current_velocity_lin = 0.0
    current_velocity_ang = 0.0
    dist = 0.0
    acc_limit_lin = 0.01
    acc_limit_ang = 0.01

    dist_list = []
    act_dist_list = []
    target_angle_old = 0.0

    #######


    while not rospy.is_shutdown() and target_reached == False:

        # calculate remaining distance to the next control point
        robot0_dist_to_cp = math.sqrt((robot0_path.y[index]-robot0_target_pose.y)**2 + (robot0_path.x[index]-robot0_target_pose.x)**2)
        robot1_dist_to_cp = math.sqrt((robot1_path.y[index]-robot1_target_pose.y)**2 + (robot1_path.x[index]-robot1_target_pose.x)**2)
        robot2_dist_to_cp = math.sqrt((robot2_path.y[index]-robot2_target_pose.y)**2 + (robot2_path.x[index]-robot2_target_pose.x)**2)

        dists_to_cp = [robot0_dist_to_cp,robot1_dist_to_cp,robot2_dist_to_cp]
        dists_to_cp.sort()
        #print(dists_to_cp)



        robot0_acc_lin = (robot0_dist_to_cp / dists_to_cp(2)) * target_vel_lin/control_rate - robot0_current_velocity_lin
        robot1_acc_lin = (robot1_dist_to_cp / dists_to_cp(2)) * target_vel_lin/control_rate - robot0_current_velocity_lin
        robot2_acc_lin = (robot2_dist_to_cp / dists_to_cp(2)) * target_vel_lin/control_rate - robot0_current_velocity_lin

        # limit acceleration
        if abs(robot0_acc_lin) > acc_limit_lin:
            robot1_acc_lin *= abs(acc_limit_lin) / robot0_acc_lin
            robot2_acc_lin *= abs(acc_limit_lin) / robot0_acc_lin
            robot0_acc_lin = robot0_acc_lin / abs(robot0_acc_lin) * acc_limit_lin

        if abs(robot1_acc_lin) > acc_limit_lin:
                robot0_acc_lin *= abs(acc_limit_lin) / robot1_acc_lin
                robot2_acc_lin *= abs(acc_limit_lin) / robot1_acc_lin
                robot1_acc_lin = robot1_acc_lin / abs(robot1_acc_lin) * acc_limit_lin

        if abs(robot2_acc_lin) > acc_limit_lin:
                robot0_acc_lin *= abs(acc_limit_lin) / robot2_acc_lin
                robot1_acc_lin *= abs(acc_limit_lin) / robot2_acc_lin
                robot2_acc_lin = robot2_acc_lin / abs(robot2_acc_lin) * acc_limit_lin

        robot0_current_velocity_lin += robot0_acc_lin/control_rate
        robot1_current_velocity_lin += robot1_acc_lin/control_rate
        robot2_current_velocity_lin += robot2_acc_lin/control_rate

        if dist + robot0_current_velocity_lin >= robot0_path_distance:
            while dist + robot0_current_velocity_lin >= robot0_path_distance and not rospy.is_shutdown():
                index += 1
                if index > len(robot0_v)-2:
                    target_reached = True
                    print("target reached")
                    break
                else:
                    robot0_path_distance += robot0_v[index]


        robot0_dist_to_cp = math.sqrt((robot0_path.y[index]-robot0_target_pose.y)**2 + (robot0_path.x[index]-robot0_target_pose.x)**2)
        if robot0_dist_to_cp<robot0_current_velocity_lin:
            index += 1
            #print("slow")
        if index >= len(robot0_path.y):
            break

        
        robot0_target_angle = math.atan2(robot0_path.y[index]-robot0_target_pose.y, robot0_path.x[index]-robot0_target_pose.x)
        robot1_target_angle = math.atan2(robot1_path.y[index]-robot1_target_pose.y, robot1_path.x[index]-robot1_target_pose.x)
        robot2_target_angle = math.atan2(robot2_path.y[index]-robot2_target_pose.y, robot2_path.x[index]-robot2_target_pose.x)

        robot0_w_target=robot0_target_angle-robot0_current_angle
        robot1_w_target=robot1_target_angle-robot1_current_angle
        robot2_w_target=robot2_target_angle-robot2_current_angle
        if abs(robot0_w_target) > w_limit:
            robot0_w_target = robot0_w_target / abs(robot0_w_target) * w_limit
        robot0_current_angle += robot0_w_target

        robot0_target_pose.x = robot0_target_pose.x + math.cos(robot0_target_angle) * robot0_current_velocity_lin
        robot0_target_pose.y = robot0_target_pose.y + math.sin(robot0_target_angle) * robot0_current_velocity_lin
            
        dist += robot0_current_velocity_lin / control_rate
        robot0_target_path.x.append(robot0_target_pose.x)
        robot0_target_path.y.append(robot0_target_pose.y)
        dist_list.append(robot0_path_distance)
        act_dist_list.append(dist)

    xhat = savgol_filter(robot0_target_path.x, 51, 3) # window size 51, polynomial order 3
    yhat = savgol_filter(robot0_target_path.y, 51, 3) # window size 51, polynomial order 3

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




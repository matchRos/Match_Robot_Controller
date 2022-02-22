#! /usr/bin/env python3

from cmath import sqrt
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from mypath import Mypath
import math
import numpy as np
from scipy.signal import savgol_filter
from matplotlib import pyplot as plt

    

def compute_trajectories(robot0_path,robot1_path,robot2_path,robot0_v,robot0_w,robot1_v,robot1_w,robot2_v,robot2_w):
    control_rate = rospy.get_param("~control_rate")
    target_vel_lin = rospy.get_param("~target_vel_lin")
    w_limit = rospy.get_param("~w_limit")


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
    robot1_current_angle = robot1_path.phi[0]
    robot2_current_angle = robot2_path.phi[0]
    index = 1
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



        robot0_acc_lin = (robot0_dist_to_cp / dists_to_cp[2]) * target_vel_lin/control_rate - robot0_current_velocity_lin
        robot1_acc_lin = (robot1_dist_to_cp / dists_to_cp[2]) * target_vel_lin/control_rate - robot1_current_velocity_lin
        robot2_acc_lin = (robot2_dist_to_cp / dists_to_cp[2]) * target_vel_lin/control_rate - robot2_current_velocity_lin

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
            robot1_w_target *= abs(w_limit) * abs(robot0_w_target)
            robot2_w_target *= abs(w_limit) * abs(robot0_w_target)
            robot0_w_target = robot0_w_target / abs(robot0_w_target) * w_limit
        robot0_current_angle += robot0_w_target

        if abs(robot1_w_target) > w_limit:
            robot0_w_target *= abs(w_limit) * abs(robot1_w_target)
            robot2_w_target *= abs(w_limit) * abs(robot1_w_target)
            robot1_w_target = robot1_w_target / abs(robot1_w_target) * w_limit
        robot1_current_angle += robot1_w_target

        if abs(robot2_w_target) > w_limit:
            robot0_w_target *= abs(w_limit) * abs(robot2_w_target)
            robot1_w_target *= abs(w_limit) * abs(robot2_w_target)
            robot2_w_target = robot2_w_target / abs(robot2_w_target) * w_limit
        robot2_current_angle += robot2_w_target

        robot0_target_pose.x = robot0_target_pose.x + math.cos(robot0_target_angle) * robot0_current_velocity_lin
        robot0_target_pose.y = robot0_target_pose.y + math.sin(robot0_target_angle) * robot0_current_velocity_lin
        robot1_target_pose.x = robot1_target_pose.x + math.cos(robot1_target_angle) * robot1_current_velocity_lin
        robot1_target_pose.y = robot1_target_pose.y + math.sin(robot1_target_angle) * robot1_current_velocity_lin
        robot2_target_pose.x = robot2_target_pose.x + math.cos(robot2_target_angle) * robot2_current_velocity_lin
        robot2_target_pose.y = robot2_target_pose.y + math.sin(robot2_target_angle) * robot2_current_velocity_lin
            
        dist += robot0_current_velocity_lin / control_rate
        robot0_target_path.x.append(robot0_target_pose.x)
        robot0_target_path.y.append(robot0_target_pose.y)
        robot1_target_path.x.append(robot1_target_pose.x)
        robot1_target_path.y.append(robot1_target_pose.y)
        robot2_target_path.x.append(robot2_target_pose.x)
        robot2_target_path.y.append(robot2_target_pose.y)


    robot0_xhat = savgol_filter(robot0_target_path.x, 51, 3) # window size 51, polynomial order 3
    robot0_yhat = savgol_filter(robot0_target_path.y, 51, 3) # window size 51, polynomial order 3
    robot1_xhat = savgol_filter(robot1_target_path.x, 51, 3) # window size 51, polynomial order 3
    robot1_yhat = savgol_filter(robot1_target_path.y, 51, 3) # window size 51, polynomial order 3
    robot2_xhat = savgol_filter(robot2_target_path.x, 51, 3) # window size 51, polynomial order 3
    robot2_yhat = savgol_filter(robot2_target_path.y, 51, 3) # window size 51, polynomial order 3


    if len(robot0_xhat) == len(robot1_xhat) == len(robot2_xhat):
        rospy.loginfo_once("All trajecories have equal length")
    else:
        rospy.logerr_once("Trajectory lenghts not equal")

  
################################################################################################################
    robot0_pub = rospy.Publisher("robot0/target_trajectory", Path, queue_size=1 )
    robot1_pub = rospy.Publisher("robot1/target_trajectory", Path, queue_size=1 )
    robot2_pub = rospy.Publisher("robot2/target_trajectory", Path, queue_size=1 )

    robot0_target_trajectory = Path()
    robot1_target_trajectory = Path()
    robot2_target_trajectory = Path()
    robot0_target_trajectory_point = PoseStamped()
    robot1_target_trajectory_point = PoseStamped()
    robot2_target_trajectory_point = PoseStamped()

    robot0_target_trajectory.header.frame_id = "map"
    robot0_target_trajectory.header.stamp = rospy.Time.now()
    robot1_target_trajectory.header.frame_id = "map"
    robot1_target_trajectory.header.stamp = rospy.Time.now()
    robot2_target_trajectory.header.frame_id = "map"
    robot2_target_trajectory.header.stamp = rospy.Time.now()

    
    robot0_target_trajectory.poses = [PoseStamped() for i in range(len(robot0_xhat))] 
    robot1_target_trajectory.poses = [PoseStamped() for i in range(len(robot1_xhat))] 
    robot2_target_trajectory.poses = [PoseStamped() for i in range(len(robot2_xhat))] 

    for i in range(0,len(robot0_xhat)): #len(robot0_xhat)
        robot0_target_trajectory_point.pose.position.x = robot0_xhat[i]
        robot0_target_trajectory_point.pose.position.y = robot0_yhat[i]
        robot1_target_trajectory_point.pose.position.x = robot1_xhat[i]
        robot1_target_trajectory_point.pose.position.y = robot1_yhat[i]
        robot2_target_trajectory_point.pose.position.x = robot2_xhat[i]
        robot2_target_trajectory_point.pose.position.y = robot2_yhat[i]

        robot0_target_trajectory.poses[i].pose.position.x = robot0_target_trajectory_point.pose.position.x
        robot0_target_trajectory.poses[i].pose.position.y = robot0_target_trajectory_point.pose.position.y
        robot1_target_trajectory.poses[i].pose.position.x = robot1_target_trajectory_point.pose.position.x
        robot1_target_trajectory.poses[i].pose.position.y = robot1_target_trajectory_point.pose.position.y
        robot2_target_trajectory.poses[i].pose.position.x = robot2_target_trajectory_point.pose.position.x
        robot2_target_trajectory.poses[i].pose.position.y = robot2_target_trajectory_point.pose.position.y

    rospy.sleep(0.1)
    robot0_pub.publish(robot0_target_trajectory)
    rospy.sleep(0.1)
    robot1_pub.publish(robot1_target_trajectory)
    rospy.sleep(0.1)
    robot2_pub.publish(robot2_target_trajectory)


    





#####################################################################################################################

    # plt.figure()
    # f, (ax1, ax2) = plt.subplots(1, 2, sharey=True)
    # ax1.plot(robot0_xhat,robot0_yhat)
    # ax2.plot(robot0_path.x,robot0_path.y)
    # ax1.plot(robot1_xhat,robot1_yhat)
    # ax2.plot(robot1_path.x,robot1_path.y)
    # ax1.plot(robot2_xhat,robot2_yhat)
    # ax2.plot(robot2_path.x,robot2_path.y)
    # plt.show()


    # robot0_v_path = [0.0]
    # robot0_phi_path = [robot0_path.phi[0]]
    # robot0_phi_path[0] = math.atan2(robot0_yhat[1]-robot0_yhat[0], robot0_xhat[1]-robot0_xhat[0])
    # robot0_w_path = [0.0]

    # robot1_v_path = [0.0]
    # robot1_phi_path = [robot1_path.phi[0]]
    # robot1_phi_path[0] = math.atan2(robot1_yhat[1]-robot1_yhat[0], robot1_xhat[1]-robot1_xhat[0])
    # robot1_w_path = [0.0]

    # robot2_v_path = [0.0]
    # robot2_phi_path = [robot1_path.phi[0]]
    # robot2_phi_path[0] = math.atan2(robot2_yhat[1]-robot2_yhat[0], robot2_xhat[1]-robot2_xhat[0])
    # robot2_w_path = [0.0]

    # for i in range(1,len(robot0_xhat)):
    #     robot0_v_path.append(math.sqrt((robot0_xhat[i]-robot0_xhat[i-1])**2+(robot0_yhat[i]-robot0_yhat[i-1])**2))
    #     robot0_phi_path.append(math.atan2(robot0_yhat[i]-robot0_yhat[i-1], robot0_xhat[i]-robot0_xhat[i-1]))
    #     robot1_v_path.append(math.sqrt((robot1_xhat[i]-robot1_xhat[i-1])**2+(robot1_yhat[i]-robot1_yhat[i-1])**2))
    #     robot1_phi_path.append(math.atan2(robot1_yhat[i]-robot1_yhat[i-1], robot1_xhat[i]-robot1_xhat[i-1]))
    #     robot2_v_path.append(math.sqrt((robot2_xhat[i]-robot2_xhat[i-1])**2+(robot2_yhat[i]-robot2_yhat[i-1])**2))
    #     robot2_phi_path.append(math.atan2(robot2_yhat[i]-robot2_yhat[i-1], robot2_xhat[i]-robot2_xhat[i-1]))
    #     #v_path.append(math.sqrt((target_path.x[i]-target_path.x[i-1])**2+(target_path.y[i]-target_path.y[i-1])**2))
    #     #phi_path.append(math.atan2(target_path.y[i]-target_path.y[i-1], target_path.x[i]-target_path.x[i-1]))
    #     robot0_w_path.append(robot0_phi_path[i]-robot0_phi_path[i-1])
    #     robot1_w_path.append(robot1_phi_path[i]-robot1_phi_path[i-1])
    #     robot2_w_path.append(robot2_phi_path[i]-robot2_phi_path[i-1])
    # plt.figure()
    # plt.plot(robot0_v_path)
    # plt.plot(robot1_v_path)
    # plt.plot(robot2_v_path)
    # plt.figure()
    # plt.plot(robot0_w_path)
    # plt.plot(robot1_w_path)
    # plt.plot(robot2_w_path)
    # #plt.plot(robot0_w_path)
    # plt.show()

    #rospy.sleep(100)

    #################################################################################################



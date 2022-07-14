#! /usr/bin/env python3
from tf import transformations
import math
import rospy




def cartesian_controller(actual_pose,target_pose,target_velocity):
    Kv = 0.5
    Ky = 0.45
    Kx = 0.3
    phi_act = transformations.euler_from_quaternion([actual_pose.orientation.x,actual_pose.orientation.y,actual_pose.orientation.z,actual_pose.orientation.w])
    phi_target = transformations.euler_from_quaternion([target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z,target_pose.orientation.w])

    e_x = (target_pose.position.x-actual_pose.position.x)
    e_y = (target_pose.position.x - actual_pose.position.y)
    rospy.loginfo_throttle(1,[id,e_x,e_y])
    e_local_x = math.cos(phi_act[2]) * e_x + math.sin(phi_act[2]) * e_y
    e_local_y = math.cos(phi_act[2]) * e_y - math.sin(phi_act[2]) * e_x

    u_w = target_velocity.angular.z + target_velocity.linear.x * Kv * e_local_y + Ky * math.sin(phi_target[2]-phi_act[2])
    u_v = target_velocity.linear.x * math.cos(phi_target[2]-phi_act[2]) + Kx*e_local_x

    return u_v, u_w

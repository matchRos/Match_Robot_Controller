#! /usr/bin/env python3
from tf import transformations
import math



def l_phi_controller(act_pose,set_pose_x,set_pose_y):
            e_x = set_pose_x - act_pose.position.x
            e_y = set_pose_y - act_pose.position.y
            phi_act = transformations.euler_from_quaternion([act_pose .orientation.x,act_pose .orientation.y,act_pose .orientation.z,act_pose .orientation.w])
            phi_target = math.atan2(e_y,e_x)
            e_phi = phi_target - phi_act[2]
            e_l = math.sqrt(e_x**2 + e_y**2)

            e_phi = e_phi % math.pi
            if e_phi > math.pi/2:
                e_phi -= e_phi
                e_l = -e_l
            elif e_phi < -math.pi/2:
                e_phi += e_phi
                e_l = -e_l

            return e_l, e_phi
#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose
import math
from tf import transformations



class Move_to_initial_pose():
    
    
    def __init__(self):
        self.config()
        rospy.init_node("lyapunov_controller_node")
        pass
    
    
    
    def main(self):
        
        Rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            phi_actual = transformations.euler_from_quaternion([self.robot2_pose.orientation.x,self.robot2_pose.orientation.y,self.robot2_pose.orientation.z,self.robot2_pose.orientation.w])
            e_phi = math.atan2(self.robot2_target_pose.position.y-self.robot2_pose.position.y,self.robot2_target_pose.position.x-self.robot2_pose.position.x) - phi_actual[2]
            
            self.command.angular.z = self.Kphi * e_phi
            print(e_phi)
            
            if abs(self.command.angular.z) > self.limit_w:
                self.command.angular.z = self.command.angular.z / abs(self.command.angular.z) * self.limit_w
            if abs(e_phi) < self.target_threshhold_angular:
                self.command.angular.z = 0
                break
            self.cmd_vel_pub.publish(self.command)
            
        while not rospy.is_shutdown():
            e_d = math.sqrt((self.robot2_target_pose.position.y-self.robot2_pose.position.y)**2+(self.robot2_target_pose.position.x-self.robot2_pose.position.x)**2)
            self.command.linear.x = self.Kd * e_d
            
            if abs(self.command.linear.x) > self.limit_x:
                self.command.linear.x = self.command.linear.x / abs(self.command.linear.x) * self.limit_x
                
            if abs(e_d) < self.target_threshhold_linear:
                self.command.linear.x = 0
                break
            self.cmd_vel_pub.publish(self.command)
            print(e_d)
            
        while not rospy.is_shutdown():
            phi_actual = transformations.euler_from_quaternion([self.robot2_pose.orientation.x,self.robot2_pose.orientation.y,self.robot2_pose.orientation.z,self.robot2_pose.orientation.w])
            phi_target = transformations.euler_from_quaternion([self.robot0_pose.orientation.x,self.robot0_pose.orientation.y,self.robot0_pose.orientation.z,self.robot0_pose.orientation.w])
            e_phi = phi_target[2] - phi_actual[2]
            
            self.command.angular.z = self.Kphi * e_phi
            print(e_phi)
            
            if abs(self.command.angular.z) > self.limit_w:
                self.command.angular.z = self.command.angular.z / abs(self.command.angular.z) * self.limit_w
            if abs(e_phi) < self.target_threshhold_angular:
                self.command.angular.z = 0
                break
            self.cmd_vel_pub.publish(self.command)
    
    def robot0_pose_cb(self,Pose):
        self.robot0_pose = Pose
        R = transformations.quaternion_matrix([Pose.orientation.x,Pose.orientation.y,Pose.orientation.z,Pose.orientation.w])
        self.robot2_target_pose.position.x = Pose.position.x + R[0][0] * self.robot2_pose_offset[0] + R[0][1] * self.robot2_pose_offset[1]
        self.robot2_target_pose.position.y = Pose.position.y + R[1][0] * self.robot2_pose_offset[0] + R[1][1] * self.robot2_pose_offset[1]
        print(self.robot2_target_pose)
        
    def robot2_pose_cb(self,Pose):
        self.robot2_pose = Pose
    
    
    def config(self):
        rospy.Subscriber("/robot0/robot_pose", Pose, self.robot0_pose_cb)
        rospy.Subscriber("/robot2/robot_pose", Pose, self.robot2_pose_cb)
        self.cmd_vel_pub = rospy.Publisher("robot2/mobile_base_controller/cmd_vel",Twist,queue_size=1)
        self.command = Twist()
        self.robot2_target_pose = Pose()
        self.control_rate = 100
        self.Kphi = 0.1
        self.Kd = 0.1
        self.target_threshhold_angular = 0.005
        self.target_threshhold_linear = 0.01
        self.robot2_pose_offset = rospy.get_param("/robot2/move_base_flex/FormationPathPlanner/formation_config/robot2/offset")
        
        self.limit_w = 0.3
        self.limit_x = 0.1
        pass
    
    
    
if __name__=="__main__":
    exe = Move_to_initial_pose()
    rospy.sleep(2)
    exe.main()
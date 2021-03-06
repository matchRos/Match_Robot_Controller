#! /usr/bin/env python3

from cmath import sqrt
import rospy
from nav_msgs.msg import Path
from mypath import Mypath
from compute_trajectories import compute_trajectories
from tf import transformations
import matplotlib.pyplot as plt
import math

class compute_velocity():



    def __init__(self):
        self.config()
        self.robot0_path = Mypath()
        self.robot1_path = Mypath()
        self.robot2_path = Mypath()
        self.robot0_path_received = False
        self.robot1_path_received = False
        self.robot2_path_received = False
        self.target_trajectory = Mypath()
        self.target_pose = Mypath()
        rospy.init_node("compute_velocity_node")
        rospy.loginfo("compute_velocity node running")
        
        rospy.Subscriber(self.robot0_plan_topic, Path, self.robot0_plan_cb)
        rospy.Subscriber(self.robot1_plan_topic, Path, self.robot1_plan_cb)
        rospy.Subscriber(self.robot2_plan_topic, Path, self.robot2_plan_cb)
        self.run()
        rospy.spin()



    def robot0_plan_cb(self,Path):
        self.robot0_path.x = []
        self.robot0_path.y = []
        self.robot0_path.phi = []
        path_len = len(Path.poses)
        for i in range(0,path_len):
            self.robot0_path.x.append(Path.poses[i].pose.position.x)
            self.robot0_path.y.append(Path.poses[i].pose.position.y)
            phi = transformations.euler_from_quaternion([Path.poses[i].pose.orientation.x,Path.poses[i].pose.orientation.y,Path.poses[i].pose.orientation.z,Path.poses[i].pose.orientation.w])
            self.robot0_path.phi.append(phi[2])
        
        self.robot0_v = [0.0]
        self.robot0_w = [0.0]
        for i in range(1,path_len-1):
            self.robot0_v.append(math.sqrt((self.robot0_path.x[i+1]-self.robot0_path.x[i])**2 + (self.robot0_path.y[i+1]-self.robot0_path.y[i])**2 ))
            self.robot0_w.append(self.robot0_path.phi[i+1]-self.robot0_path.phi[i])
        self.robot0_path_received = True

    def robot1_plan_cb(self,Path):
        self.robot1_path.x = []
        self.robot1_path.y = []
        self.robot1_path.phi = []
        path_len = len(Path.poses)
        for i in range(0,path_len):
            self.robot1_path.x.append(Path.poses[i].pose.position.x)
            self.robot1_path.y.append(Path.poses[i].pose.position.y)
            phi = transformations.euler_from_quaternion([Path.poses[i].pose.orientation.x,Path.poses[i].pose.orientation.y,Path.poses[i].pose.orientation.z,Path.poses[i].pose.orientation.w])
            self.robot1_path.phi.append(phi[2])
        
        self.robot1_v = [0.0]
        self.robot1_w = [0.0]
        for i in range(1,path_len-1):
            self.robot1_v.append(math.sqrt((self.robot1_path.x[i+1]-self.robot1_path.x[i])**2 + (self.robot1_path.y[i+1]-self.robot1_path.y[i])**2 ))
            self.robot1_w.append(self.robot1_path.phi[i+1]-self.robot1_path.phi[i])
        self.robot1_path_received = True

    def robot2_plan_cb(self,Path):
        self.robot2_path.x = []
        self.robot2_path.y = []
        self.robot2_path.phi = []
        path_len = len(Path.poses)
        for i in range(0,path_len):
            self.robot2_path.x.append(Path.poses[i].pose.position.x)
            self.robot2_path.y.append(Path.poses[i].pose.position.y)
            phi = transformations.euler_from_quaternion([Path.poses[i].pose.orientation.x,Path.poses[i].pose.orientation.y,Path.poses[i].pose.orientation.z,Path.poses[i].pose.orientation.w])
            self.robot2_path.phi.append(phi[2])
        
        self.robot2_v = [0.0]
        self.robot2_w = [0.0]
        for i in range(1,path_len-1):
            self.robot2_v.append(math.sqrt((self.robot2_path.x[i+1]-self.robot2_path.x[i])**2 + (self.robot2_path.y[i+1]-self.robot2_path.y[i])**2 ))
            self.robot2_w.append(self.robot2_path.phi[i+1]-self.robot2_path.phi[i])
        self.robot2_path_received = True




    def run(self):
        self.target_trajectory.x = []
        self.target_trajectory.y = []
        self.target_trajectory.z = []


        while not rospy.is_shutdown():
            if self.robot0_path_received == True & self.robot1_path_received == True & self.robot1_path_received == True:
                rospy.loginfo_once("received all paths")
                break
            else:
                rospy.loginfo("waiting for paths")
                rospy.sleep(1)

        compute_trajectories(self.robot0_path,self.robot1_path,self.robot2_path,self.robot0_v,self.robot0_w,self.robot1_v,self.robot1_w,self.robot2_v,self.robot2_w)
                # set start pose
        self.target_pose.x = self.robot0_path.x[0]
        self.target_pose.y = self.robot0_path.y[0]
        self.target_pose.phi = self.robot0_path.phi[0]
        self.target_pose.x = self.target_pose.x
        
        print("done")



    def config(self):
        self.robot0_plan_topic = rospy.get_param('~robot0_plan_topic',"/robot0/move_base_flex/plan")
        self.robot1_plan_topic = rospy.get_param('~robot1_plan_topic',"/robot1/move_base_flex/plan")
        self.robot2_plan_topic = rospy.get_param('~robot2_plan_topic',"/robot2/move_base_flex/plan")
        self.control_rate = rospy.get_param('~control_rate',100)




if __name__=="__main__":
    compute_velocity()

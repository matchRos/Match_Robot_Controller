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



    def plan_cb(self,Path,robot_id):
        robot_path = Mypath()
        robot_path.x = []
        robot_path.y = []
        robot_path.phi = []
        robot_path.v = [0.0]
        robot_path.w = [0.0]

        path_len = len(Path.poses)
        for i in range(0,path_len):
            robot_path.x.append(Path.poses[i].pose.position.x)
            robot_path.y.append(Path.poses[i].pose.position.y)
            phi = transformations.euler_from_quaternion([Path.poses[i].pose.orientation.x,Path.poses[i].pose.orientation.y,Path.poses[i].pose.orientation.z,Path.poses[i].pose.orientation.w])
            robot_path.phi.append(phi[2])
        
        for i in range(1,path_len-1):
            robot_path.w.append(math.sqrt((robot_path.x[i+1]-robot_path.x[i])**2 + (robot_path.y[i+1]-robot_path.y[i])**2 ))
            robot_path.w.append(robot_path.phi[i+1]-robot_path.phi[i])
        
        self.robot_paths[robot_id] = robot_path
        self.robot0_path_received = True



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

        self.robot_paths = []
 
 
        self.number_of_robots = rospy.get_param("~number_of_robots")
        for i in range(0,self.number_of_robots):
            param = "~robot" + str(i) + "_plan_topic"
            robotX_plan_topic = rospy.get_param(param)
            rospy.Subscriber(robotX_plan_topic, Path, self.plan_cb, i)

            self.robot_paths.append(Mypath())


        self.robot0_plan_topic = rospy.get_param('~robot0_plan_topic',"/robot0/move_base_flex/plan")
        self.robot1_plan_topic = rospy.get_param('~robot1_plan_topic',"/robot1/move_base_flex/plan")
        self.robot2_plan_topic = rospy.get_param('~robot2_plan_topic',"/robot2/move_base_flex/plan")
        self.control_rate = rospy.get_param('~control_rate',100)




if __name__=="__main__":
    compute_velocity()

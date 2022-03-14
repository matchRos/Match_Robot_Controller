#! /usr/bin/env python3

from cmath import sqrt
import rospy
from nav_msgs.msg import Path
from mypath import Mypath
from compute_trajectories import compute_trajectories
from tf import transformations
import math

class compute_velocity():



    def __init__(self):
        rospy.init_node("compute_velocity_node")
        rospy.loginfo("compute_velocity node running")
        self.config()
        self.robot0_path = Mypath()
        self.robot1_path = Mypath()
        self.robot2_path = Mypath()
        self.robot0_path_received = False
        self.robot1_path_received = False
        self.robot2_path_received = False
        self.target_trajectory = Mypath()
        self.target_pose = Mypath()

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
            robot_path.v.append(math.sqrt((robot_path.x[i+1]-robot_path.x[i])**2 + (robot_path.y[i+1]-robot_path.y[i])**2 ))
            robot_path.w.append(robot_path.phi[i+1]-robot_path.phi[i])
        
        self.robot_paths[robot_id] = robot_path
        self.robotX_path_received[robot_id] = True



    def run(self):
        self.target_trajectory.x = []
        self.target_trajectory.y = []
        self.target_trajectory.z = []


        while not rospy.is_shutdown():
            number_of_plaths_reveived = 0
            for i in range(0,self.number_of_robots):
                if self.robotX_path_received[i] == True:
                    number_of_plaths_reveived += 1
            
            if number_of_plaths_reveived == self.number_of_robots:
                rospy.loginfo_once("received all paths")
                break
            else:
                # rospy.loginfo("waiting for paths: ", number_of_plaths_reveived, " of ", self.number_of_robots)
                print("waiting for paths: ", number_of_plaths_reveived, " of ", self.number_of_robots)
                rospy.sleep(1)

        compute_trajectories(self.robot_paths[0],self.robot_paths[1],self.robot_paths[2],self.robot_paths[0].v)

        
        print("done")



    def config(self):

        self.robot_paths = []
        self.robotX_path_received = []
 
 
        self.number_of_robots = rospy.get_param("~number_of_robots")
        for i in range(0,self.number_of_robots):
            param = "~robot" + str(i) + "_plan_topic"
            robotX_plan_topic = rospy.get_param(param)
            rospy.Subscriber(robotX_plan_topic, Path, self.plan_cb, i)

            self.robot_paths.append(Mypath())
            self.robotX_path_received.append(False)

        self.control_rate = rospy.get_param('~control_rate',100)




if __name__=="__main__":
    compute_velocity()

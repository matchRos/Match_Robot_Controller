#! /usr/bin/env python3

import rospy
#from nav_msgs.msg import Path
from mypath import Mypath
from nav_msgs.msg import Path
import math
from geometry_msgs.msg import Twist



class execute_trajectories_node():

    def __init__(self):
        rospy.init_node("lyapunov_controller_node")
        rospy.loginfo("lyapunov controller running")
        self.config()





    def run(self):
        while self.robot0_trajectory_received == False and not rospy.is_shutdown():
            rospy.sleep(0.1)
            
            
        print("all trajecories received")

        idx = 0
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and idx < len(self.robot0_v):
            self.robot0_twist.linear.x = self.robot0_v[idx]
            self.robot0_twist.angular.z = self.robot0_w[idx]
            self.robot0_twist_publisher.publish(self.robot0_twist)
            idx += 1
            rate.sleep()



    def robot0_trajectory_cb(self,Path):
        self.robot0_trajectory.x = []
        self.robot0_trajectory.y = []
        self.robot0_trajectory.phi = []
        path_len = len(Path.poses)
        for i in range(0,path_len-1):
            self.robot0_trajectory.x.append(Path.poses[i].pose.position.x)
            self.robot0_trajectory.y.append(Path.poses[i].pose.position.y)
            phi = math.atan2(Path.poses[i+1].pose.position.y-Path.poses[i].pose.position.y,Path.poses[i+1].pose.position.x-Path.poses[i].pose.position.x)
            self.robot0_trajectory.phi.append(phi)
        

        self.robot0_v = [0.0]
        self.robot0_w = [0.0]
        for i in range(1,path_len-2):
            self.robot0_v.append(math.sqrt((self.robot0_trajectory.x[i+1]-self.robot0_trajectory.x[i])**2 + (self.robot0_trajectory.y[i+1]-self.robot0_trajectory.y[i])**2 ))
            self.robot0_w.append(self.robot0_trajectory.phi[i+1]-self.robot0_trajectory.phi[i])
        self.robot0_trajectory_received = True
        print("trajecory 1 received")
        #print(self.robot0_v)









    def config(self):
        robot0_trajectory_topic = "robot0/target_trajectory"
        robot1_trajectory_topic = "robot0/target_trajectory"
        robot2_trajectory_topic = "robot0/target_trajectory"
        rospy.Subscriber(robot0_trajectory_topic, Path, self.robot0_trajectory_cb)
        #rospy.Subscriber(robot1_trajectory_topic, Path, self.robot1_path_cb)
        #rospy.Subscriber(robot2_trajectory_topic, Path, self.robot2_path_cb)
        self.robot0_trajectory = Mypath()
        self.robot1_trajectory = Mypath()
        self.robot2_trajectory = Mypath()
        self.robot0_twist_publisher = rospy.Publisher("/robot0/mobile_base_controller/cmd_vel2",Twist,queue_size=5)
        self.robot0_twist = Twist()
        self.robot0_trajectory_received = False




if __name__=="__main__":
    exe = execute_trajectories_node()
    exe.run()

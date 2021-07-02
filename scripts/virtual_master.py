#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Twist
from nav_msgs.msg import Odometry
import tf
import geometry_msgs
import math
from tf import transformations
from rostopic import get_topic_type


class lyapunov_controller_node:

    def __init__(self):
        rospy.init_node("virtual_master_node")
        rospy.loginfo("virtual_master_node running")
        self.config()
        self.time_old = rospy.get_time()
        self.master_pose = Pose()
        self.master_vel = Twist()
        self.d_pose = [0,0,0]
        self.d_pose_R = [0,0,0]
        self.master_orientation = 0




        rospy.Subscriber(self.set_pose_topic, Pose, self.set_pose_cb)
        rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_vel_cb)
        self.pub        = rospy.Publisher(self.master_pose_topic, Pose, queue_size=10)
        self.pub_vel    = rospy.Publisher(self.master_vel_topic, Twist, queue_size=10)
        self.run()

        rospy.spin()


    def config(self):
        self.rate = rospy.get_param('~rate')
        self.set_pose_topic = rospy.get_param('~set_pose_topic')
        self.master_vel_topic = rospy.get_param('~master_vel_topic')
        self.master_pose_topic = rospy.get_param('~master_pose_topic')
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic')
        
    def run(self):
        Rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            time_current = rospy.get_time()
            duration = time_current - self.time_old
            self.time_old = time_current
            
            self.d_pose[0] = self.master_vel.linear.x * duration
            self.d_pose[1] = self.master_vel.linear.y * duration
            self.d_pose[2] = self.master_vel.angular.z * duration
            
            R = transformations.euler_matrix(0,0,self.master_orientation)
            
            self.d_pose_R[0] = R[0,0] * self.d_pose[0] + R[0,1] * self.d_pose[1] 
            self.d_pose_R[1] = R[1,0] * self.d_pose[0] + R[1,1] * self.d_pose[1] 
            self.d_pose_R[2] = self.d_pose[2]
            
            self.master_pose.position.x = self.master_pose.position.x + self.d_pose_R[0]
            self.master_pose.position.y = self.master_pose.position.y + self.d_pose_R[1]
            self.master_orientation = self.master_orientation + self.d_pose_R[2]
            
            q = transformations.quaternion_from_euler(0,0,self.master_orientation)
            self.master_pose.orientation.x = q[0]
            self.master_pose.orientation.y = q[1]
            self.master_pose.orientation.z = q[2]
            self.master_pose.orientation.w = q[3]
            
            self.pub.publish(self.master_pose)
            self.pub_vel.publish(self.master_vel)
            
            Rate.sleep()

    def set_pose_cb(self,data):
        self.master_pose = data
        orientation = transformations.euler_from_quaternion([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w])
        self.master_orientation = orientation[2]
        

    def cmd_vel_cb(self,data):
        self.master_vel = data
       


if __name__=="__main__":
    lyapunov_controller_node()

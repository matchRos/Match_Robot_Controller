#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Twist
from nav_msgs.msg import Odometry
import tf
import geometry_msgs
import math
from tf import transformations


class lyapunov_controller_node:

    def __init__(self):
        rospy.init_node("lyapunov_controller_node")
        rospy.loginfo("lyapunov controller running")
        self.config()
        self.act_pose = Odometry()
        self.target_pose = Pose()
        self.target_pose = float("Inf")

        self.controller_out = Twist()


        # AMCL
        #rospy.Subscriber("/mir1/amcl_pose", PoseWithCovarianceStamped, self.act_pose_cb)
        rospy.Subscriber(self.follower_pose_topic, Odometry, self.act_pose_cb)
        rospy.Subscriber(self.master_pose_topic, Odometry, self.target_pose_cb)
        rospy.Subscriber(self.master_vel_topic, Odometry, self.target_vel_cb)
        self.pub = rospy.Publisher(self.follower_cmd_vel_topic, Twist, queue_size=10)

        rospy.spin()


    def config(self):
        self.Kx = rospy.get_param('~Kx')
        self.Ky = rospy.get_param('~Ky')
        self.Kw = rospy.get_param('~Kw')
        self.master_pose_topic = rospy.get_param('~target_pose_topic')
        self.master_vel_topic = rospy.get_param('~target_vel_topic')
        self.follower_pose_topic = rospy.get_param('~actual_pose_topic')
        self.follower_cmd_vel_topic = rospy.get_param('~cmd_vel_topic')
        self.rel_pose = rospy.get_param('~rel_pose')


    def act_pose_cb(self,data):
        if self.target_pose != float("Inf"):
            act_pose = data.pose.pose
            #print((act_pose.orientation.x,act_pose.orientation.y,act_pose.orientation.z,act_pose.orientation.w))
            act_w = tf.transformations.euler_from_quaternion((act_pose.orientation.x,act_pose.orientation.y,act_pose.orientation.z,act_pose.orientation.w))
            target_w = tf.transformations.euler_from_quaternion((self.target_pose.orientation.x,self.target_pose.orientation.y,self.target_pose.orientation.z,self.target_pose.orientation.w))
            self.e_w = target_w[2]- act_w[2]
            

            R_act = transformations.euler_matrix(act_w[0],act_w[1],-act_w[2])
            R_target = transformations.euler_matrix(target_w[0],target_w[1],target_w[2])

            rel = [0.0,0.0]
            rel[0] = R_target[0,0] * self.rel_pose[0] + R_target[0,1] * self.rel_pose[1] 
            rel[1] = R_target[1,0] * self.rel_pose[0] + R_target[1,1] * self.rel_pose[1] 

            
            ex = self.target_pose.position.x - act_pose.position.x - rel[0]
            ey = self.target_pose.position.y - act_pose.position.y - rel[1]


            self.e_x = R_act[0,0] * ex + R_act[0,1] * ey 
            self.e_y = R_act[1,0] * ex + R_act[1,1] * ey

            print(self.e_x,self.e_y,self.e_w)


            self.v_d = math.sqrt(self.target_vel.linear.x**2 + self.target_vel.linear.y**2)
            #self.v_d = 0.01
            #self.w_d = 0.1
            self.w_d = self.target_vel.angular.z

            self.controller_out.linear.x = self.Kx * self.e_x + self.v_d * math.cos(self.e_w)
            self.controller_out.angular.z = self.w_d + self.Ky  * self.e_y + self.Kw * math.sin(self.e_w)
            self.pub.publish(self.controller_out)

            
        else:
            rospy.loginfo("no target pose")

    def target_pose_cb(self,data):
        self.target_pose = data.pose.pose
        

    def target_vel_cb(self,data):
        self.target_vel = data.twist.twist


if __name__=="__main__":
    lyapunov_controller_node()

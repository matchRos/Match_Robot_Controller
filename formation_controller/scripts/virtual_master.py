#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Twist, PoseStamped, TransformStamped, Transform, TwistStamped
from nav_msgs.msg import Odometry
import tf
import geometry_msgs
import math
from tf import transformations
from rostopic import get_topic_type
import tf2_ros

class virtual_leader:

    def __init__(self):
        rospy.init_node("virtual_leader_node")
        rospy.loginfo("virtual_leader_node running")
        self.config()
        self.time_old = rospy.get_time()
        self.leader_pose = PoseStamped()
        self.master_vel = TwistStamped()
        self.d_pose = [0,0,0]
        self.d_pose_R = [0,0,0]
        self.leader_orientation = 0.0

        rospy.Subscriber(self.set_pose_topic, PoseStamped, self.set_pose_cb)
        rospy.Subscriber(self.cmd_vel_topic, TwistStamped, self.cmd_vel_cb)
        self.pub        = rospy.Publisher(self.leader_pose_topic, PoseStamped, queue_size=10)
        self.pub_vel    = rospy.Publisher(self.leader_vel_topic, TwistStamped, queue_size=10)
        self.run()

        rospy.spin()


    def config(self):
        self.rate = rospy.get_param('~rate')
        self.set_pose_topic = rospy.get_param('~set_pose_topic','/set_pose')
        self.leader_vel_topic = rospy.get_param('~leader_vel_topic','/leader_vel')
        self.leader_pose_topic = rospy.get_param('~leader_pose_topic','/leader_pose')
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic','/cmd_vel')
        
    def run(self):
        Rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            time_current = rospy.get_time()
            dt = time_current - self.time_old
            self.time_old = time_current
            
            # calculate the distance the leader has moved since the last time
            self.d_pose[0] = self.master_vel.twist.linear.x * dt
            self.d_pose[1] = self.master_vel.twist.linear.y * dt
            self.d_pose[2] = self.master_vel.twist.angular.z * dt
            
            # transform the distance to the global frame
            R = transformations.euler_matrix(0,0,self.leader_orientation)
            self.d_pose_R[0] = R[0,0] * self.d_pose[0] + R[0,1] * self.d_pose[1] 
            self.d_pose_R[1] = R[1,0] * self.d_pose[0] + R[1,1] * self.d_pose[1] 
            self.d_pose_R[2] = self.d_pose[2]
            
            # calculate the new pose of the leader
            self.leader_pose.pose.position.x = self.leader_pose.pose.position.x + self.d_pose_R[0]
            self.leader_pose.pose.position.y = self.leader_pose.pose.position.y + self.d_pose_R[1]
            self.leader_orientation = self.leader_orientation + self.d_pose_R[2]
            
            q = transformations.quaternion_from_euler(0,0,self.leader_orientation)
            self.leader_pose.pose.orientation.x = q[0]
            self.leader_pose.pose.orientation.y = q[1]
            self.leader_pose.pose.orientation.z = q[2]
            self.leader_pose.pose.orientation.w = q[3]
            
            self.leader_pose.header.stamp = rospy.Time.now()
            self.master_vel.header.stamp = rospy.Time.now()
            
            self.pub.publish(self.leader_pose)
            self.pub_vel.publish(self.master_vel)
            
            
            br = tf2_ros.TransformBroadcaster()
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "map"
            t.child_frame_id = "virtual_leader/base_footprint"
            t.transform.translation = self.leader_pose.pose.position
            t.transform.rotation = self.leader_pose.pose.orientation
            br.sendTransform(t)
            
            Rate.sleep()

    def set_pose_cb(self,data):
        self.leader_pose = data
        print(data)
        orientation = transformations.euler_from_quaternion([data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w])
        self.leader_orientation = orientation[2]
        

    def cmd_vel_cb(self,data):
        self.master_vel = data
       


if __name__=="__main__":
    virtual_leader()

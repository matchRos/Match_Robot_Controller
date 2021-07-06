#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped


from tf import transformations

class set_virtual_master_pose:

    def __init__(self):
        rospy.init_node("set_virtual_master_pose_node")
        rospy.loginfo("virtual master pose set")
        self.config()
        self.master_pose = PoseStamped()
        

        rospy.Subscriber(self.follower_pose_topic, PoseStamped, self.set_pose_cb)
        self.pub        = rospy.Publisher(self.set_pose_topic, PoseStamped, queue_size=10)

        rospy.sleep(1)


    def config(self):
        self.set_pose_topic = rospy.get_param('~set_pose_topic')
        self.follower_pose_topic = rospy.get_param('~follower_pose_topic')
        self.rel_pose = rospy.get_param('~rel_pose')
        

       

    def set_pose_cb(self,data):
        
        R = transformations.quaternion_matrix([data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w])
        self.master_pose.pose.position.x = data.pose.position.x + R[0,0] * self.rel_pose[0]  + R[0,1] * self.rel_pose[1]
        self.master_pose.pose.position.y = data.pose.position.y + R[1,0] * self.rel_pose[0]  + R[1,1] * self.rel_pose[1]
        self.master_pose.pose.orientation = data.pose.orientation

        self.pub.publish(self.master_pose)


       


if __name__=="__main__":
    set_virtual_master_pose()

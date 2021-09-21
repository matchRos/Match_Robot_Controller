#! /usr/bin/env python

from operator import mod
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Twist, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
import tf
import geometry_msgs
import math
from tf import transformations
from rostopic import get_topic_type


class lyapunov_controller_node:

    def __init__(self):
        rospy.init_node("lyapunov_controller_node")
        rospy.loginfo("lyapunov controller running")
        self.config()
        self.target_pose = Pose()
        self.target_pose = float("Inf")
        self.controller_out = TwistStamped()
        self.target_vel = Twist()
        self.e_x_i = 0
        self.e_y_i = 0
        self.e_w_i = 0

        self.callback_selector()
        rospy.sleep(0.1)
        rospy.Subscriber(self.follower_pose_topic, PoseStamped, self.act_pose_cb)
        
        self.pub = rospy.Publisher(self.follower_cmd_vel_topic, TwistStamped, queue_size=10)

        rospy.spin()


    def config(self):
        self.KPx = rospy.get_param('~KPx')
        self.KPy = rospy.get_param('~KPy')
        self.KPw = rospy.get_param('~KPw')
        self.KIw = rospy.get_param('~KIw')
        self.KIx = rospy.get_param('~KIx')
        self.KIx_decay = rospy.get_param('~KIx_decay')
        self.KIw_decay = rospy.get_param('~KIw_decay')

        self.master_pose_topic = rospy.get_param('~target_pose_topic')
        self.master_vel_topic = rospy.get_param('~target_vel_topic')
        self.follower_pose_topic = rospy.get_param('~actual_pose_topic')
        self.follower_cmd_vel_topic = rospy.get_param('~cmd_vel_topic')
        self.rel_pose = rospy.get_param('~rel_pose')
        self.max_vel = rospy.get_param('~max_vel')
        self.max_w = rospy.get_param('~max_w')
                
               


    def act_pose_cb(self,data):
        if self.target_pose != float("Inf"):
            act_pose = data.pose
            act_w = tf.transformations.euler_from_quaternion((act_pose.orientation.x,act_pose.orientation.y,act_pose.orientation.z,act_pose.orientation.w))
            target_w = tf.transformations.euler_from_quaternion((self.target_pose.orientation.x,self.target_pose.orientation.y,self.target_pose.orientation.z,self.target_pose.orientation.w))
            R_act = transformations.euler_matrix(act_w[0],act_w[1],-act_w[2])
            R_target = transformations.euler_matrix(target_w[0],target_w[1],target_w[2])

            rel = [0,0,0]
            rel[0] = R_target[0,0] * self.rel_pose[0] + R_target[0,1] * self.rel_pose[1] 
            rel[1] = R_target[1,0] * self.rel_pose[0] + R_target[1,1] * self.rel_pose[1] 

            
            ex = self.target_pose.position.x - act_pose.position.x - rel[0] # error in x (global frame)
            ey = self.target_pose.position.y - act_pose.position.y - rel[1] # error in y (global frame)
            e_w = target_w[2]- act_w[2]    # angular error
            e_x = R_act[0,0] * ex + R_act[0,1] * ey #  error in x (local frame)
            e_y = R_act[1,0] * ex + R_act[1,1] * ey #  error in y (local frame)
            self.e_x_i = (1-self.KIx_decay)*self.e_x_i + e_x
            self.e_w_i = (1-self.KIw_decay)*self.e_w_i + e_w

            v_d = math.sqrt(self.target_vel.linear.x**2 + self.target_vel.linear.y**2)
            w_d = self.target_vel.angular.z
            
            
            e_w = e_w % (2*math.pi) 
            # if(e_w>=0.5*math.pi):
            #      e_w = e_w - math.pi;
            #      v_d = -v_d;    
            # elif(e_w<= -.5*math.pi):
            #      e_w = e_w + math.pi;
            #      v_d = -v_d;            
            
            self.controller_out.twist.linear.x = self.KPx * e_x + v_d * math.cos(e_w) + self.KIx * self.e_x_i
            self.controller_out.twist.angular.z = w_d + self.KPy * v_d * e_y + self.KPw * math.sin(e_w) + self.KIw * self.e_w_i 
            self.controller_out.header.stamp = rospy.get_rostime()

            if self.controller_out.twist.linear.x > self.max_vel:
                self.controller_out.twist.linear.x = self.max_vel
            elif self.controller_out.twist.linear.x < -self.max_vel:
                self.controller_out.twist.linear.x = -self.max_vel

            if self.controller_out.twist.angular.z > self.max_w:
                self.controller_out.twist.angular.z = self.max_w
            elif self.controller_out.twist.angular.z < -self.max_w:
                self.controller_out.twist.angular.z = -self.max_w

            self.pub.publish(self.controller_out)

            
        else:
            rospy.loginfo_throttle(2, "no target pose")

    
    def callback_selector(self):
                
        topic_type =get_topic_type(self.master_pose_topic)
        if topic_type[0] == "geometry_msgs/Pose":
            rospy.Subscriber(self.master_pose_topic, Pose, self.target_pose_Pose_cb)
        if topic_type[0] == "geometry_msgs/PoseStamped":
                rospy.Subscriber(self.master_pose_topic, PoseStamped, self.target_pose_Pose_Stamped_cb)
        elif topic_type[0] == "nav_msgs/Odometry":
            rospy.Subscriber(self.master_pose_topic, Odometry, self.target_pose_cb)
            
        topic_type =get_topic_type(self.master_vel_topic)
        if topic_type[0] == "geometry_msgs/Twist":
            rospy.Subscriber(self.master_vel_topic, Twist, self.target_vel_Twist_cb)
        if topic_type[0] == "geometry_msgs/TwistStamped":
                rospy.Subscriber(self.master_vel_topic, TwistStamped, self.target_vel_Twist_Stamped_cb)
        elif topic_type[0] == "nav_msgs/Odometry":
            rospy.Subscriber(self.master_vel_topic, Odometry, self.target_vel_cb)
            
    
    def target_pose_cb(self,data):
        self.target_pose = data.pose.pose
        
    def target_pose_Pose_cb(self,data):
        self.target_pose = data
        
    def target_pose_Pose_Stamped_cb(self,data):
        self.target_pose = data.pose
           
    def target_vel_Twist_cb(self,data):
        self.target_vel = data
        
    def target_vel_cb(self,data):
        self.target_vel = data.twist.twist
        
    def target_vel_Twist_Stamped_cb(self,data):
        self.target_vel = data.twist


if __name__=="__main__":
    lyapunov_controller_node()

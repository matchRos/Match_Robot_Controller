#! /usr/bin/env python3

import rospy
import tf
from mypath import Mypath
from nav_msgs.msg import Path
import math
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Pose
from nav_msgs.msg import Odometry
from tf import transformations
from l_phi_controller import l_phi_controller
from cartesian_controller import cartesian_controller



class execute_trajectories_node():

    def __init__(self):
        rospy.init_node("lyapunov_controller_node")
        rospy.loginfo("lyapunov controller running")
        self.config()
        




    def run(self):
        while not self.robot0_trajectory_received == True and not self.robot1_trajectory_received == True and not self.robot2_trajectory_received == True and not rospy.is_shutdown():
            rospy.sleep(0.1)
            
        rospy.sleep(1)    
        print("all trajecories received")

        e0_l = 10.0
        e0_phi = 999999999.0
        rate = rospy.Rate(self.control_rate)
        idx = 0
        while (abs(e0_l) > 1.05 or abs(e0_phi) > 0.06) and not rospy.is_shutdown():
            e0_l, e0_phi = l_phi_controller(self.robot0_act_pose,self.robot0_trajectory.x[idx],self.robot0_trajectory.y[idx])
            self.robot0_twist.linear.x = 0.15 * e0_l
            self.robot0_twist.angular.z = 0.01 * e0_phi
            self.robot0_twist_publisher.publish(self.robot0_twist)

            e1_l, e1_phi = l_phi_controller(self.robot1_act_pose,self.robot1_trajectory.x[idx],self.robot1_trajectory.y[idx])
            self.robot1_twist.linear.x = 0.15 * e1_l
            self.robot1_twist.angular.z = 0.01 * e1_phi
            self.robot1_twist_publisher.publish(self.robot1_twist)

            e2_l, e2_phi = l_phi_controller(self.robot2_act_pose,self.robot2_trajectory.x[idx],self.robot2_trajectory.y[idx])
            self.robot2_twist.linear.x = 0.15 * e2_l
            self.robot2_twist.angular.z = 0.01 * e2_phi
            self.robot2_twist_publisher.publish(self.robot2_twist)

            #print(e0_l,e1_l,e1_phi,e2_l,e2_phi)
            #print(self.robot0_act_pose,self.robot0_trajectory.x[idx],self.robot0_trajectory.y[idx])


        idx = 0
        while not rospy.is_shutdown() and idx < len(self.robot0_v):
         
            u0_v, u0_w = cartesian_controller(self.robot0_act_pose,self.robot0_trajectory.x[idx],self.robot0_trajectory.y[idx],self.control_rate*self.robot0_w[idx],self.control_rate*self.robot0_v[idx],self.robot0_trajectory.phi[idx],1)
            u1_v, u1_w = cartesian_controller(self.robot1_act_pose,self.robot1_trajectory.x[idx],self.robot1_trajectory.y[idx],self.control_rate*self.robot1_w[idx],self.control_rate*self.robot1_v[idx],self.robot1_trajectory.phi[idx],1)
            u2_v, u2_w = cartesian_controller(self.robot2_act_pose,self.robot2_trajectory.x[idx],self.robot2_trajectory.y[idx],self.control_rate*self.robot2_w[idx],self.control_rate*self.robot2_v[idx],self.robot2_trajectory.phi[idx],1)

            self.robot0_twist.linear.x = u0_v   # self.robot0_v[idx] * self.control_rate
            self.robot0_twist.angular.z = u0_w      # self.robot0_w[idx] * self.control_rate
            self.robot0_twist_publisher.publish(self.robot0_twist)

            self.robot1_twist.linear.x = u1_v       #self.robot1_w[idx] * self.control_rate
            self.robot1_twist.angular.z = u1_w      #self.robot1_w[idx] * self.control_rate
            self.robot1_twist_publisher.publish(self.robot1_twist)

            self.robot2_twist.linear.x = u2_v       # self.robot2_v[idx] * self.control_rate
            self.robot2_twist.angular.z = u2_w      #self.robot2_w[idx] * self.control_rate
            self.robot2_twist_publisher.publish(self.robot2_twist)

            self.target_pose_broadcaster([self.robot0_trajectory.x[idx],self.robot0_trajectory.y[idx],self.robot0_trajectory.phi[idx]], \
                [self.robot1_trajectory.x[idx],self.robot1_trajectory.y[idx],self.robot1_trajectory.phi[idx]], \
                [self.robot2_trajectory.x[idx],self.robot2_trajectory.y[idx],self.robot2_trajectory.phi[idx]])

            self.actual_pose_broadcaster(self.robot0_act_pose,self.robot1_act_pose,self.robot2_act_pose)

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
        print("trajecory 0 received")

    def robot1_trajectory_cb(self,Path):
        self.robot1_trajectory.x = []
        self.robot1_trajectory.y = []
        self.robot1_trajectory.phi = []
        path_len = len(Path.poses)
        for i in range(0,path_len-1):
            self.robot1_trajectory.x.append(Path.poses[i].pose.position.x)
            self.robot1_trajectory.y.append(Path.poses[i].pose.position.y)
            phi = math.atan2(Path.poses[i+1].pose.position.y-Path.poses[i].pose.position.y,Path.poses[i+1].pose.position.x-Path.poses[i].pose.position.x)
            self.robot1_trajectory.phi.append(phi)
        

        self.robot1_v = [0.0]
        self.robot1_w = [0.0]
        for i in range(1,path_len-2):
            self.robot1_v.append(math.sqrt((self.robot1_trajectory.x[i+1]-self.robot1_trajectory.x[i])**2 + (self.robot1_trajectory.y[i+1]-self.robot1_trajectory.y[i])**2 ))
            self.robot1_w.append(self.robot1_trajectory.phi[i+1]-self.robot1_trajectory.phi[i])
        self.robot1_trajectory_received = True
        print("trajecory 1 received")

    def robot2_trajectory_cb(self,Path):
        self.robot2_trajectory.x = []
        self.robot2_trajectory.y = []
        self.robot2_trajectory.phi = []
        path_len = len(Path.poses)
        for i in range(0,path_len-1):
            self.robot2_trajectory.x.append(Path.poses[i].pose.position.x)
            self.robot2_trajectory.y.append(Path.poses[i].pose.position.y)
            phi = math.atan2(Path.poses[i+1].pose.position.y-Path.poses[i].pose.position.y,Path.poses[i+1].pose.position.x-Path.poses[i].pose.position.x)
            self.robot2_trajectory.phi.append(phi)
        

        self.robot2_v = [0.0]
        self.robot2_w = [0.0]
        for i in range(1,path_len-2):
            self.robot2_v.append(math.sqrt((self.robot2_trajectory.x[i+1]-self.robot2_trajectory.x[i])**2 + (self.robot2_trajectory.y[i+1]-self.robot2_trajectory.y[i])**2 ))
            self.robot2_w.append(self.robot2_trajectory.phi[i+1]-self.robot2_trajectory.phi[i])
        self.robot2_trajectory_received = True
        print("trajecory 2 received")

    def robot0_pose_cb(self,Pose):
        self.robot0_act_pose = Pose

    def robot1_pose_cb(self,Pose):
        self.robot1_act_pose = Pose

    def robot2_pose_cb(self,Pose):
        self.robot2_act_pose = Pose



    def target_pose_broadcaster(self,robot0_target_pose,robot1_target_pose,robot2_target_pose):
        self.robot0_target_pose_broadcaster.sendTransform((robot0_target_pose[0], robot0_target_pose[1], 0),
                     tf.transformations.quaternion_from_euler(0, 0, robot0_target_pose[2]),
                     rospy.Time.now(), "robot0/target_pose", "map")
        self.robot1_target_pose_broadcaster.sendTransform((robot1_target_pose[0], robot1_target_pose[1], 0),
                     tf.transformations.quaternion_from_euler(0, 0, robot1_target_pose[2]),
                     rospy.Time.now(), "robot1/target_pose", "map")
        self.robot2_target_pose_broadcaster.sendTransform((robot2_target_pose[0], robot2_target_pose[1], 0),
                     tf.transformations.quaternion_from_euler(0, 0, robot2_target_pose[2]),
                     rospy.Time.now(), "robot2/target_pose", "map")


    def actual_pose_broadcaster(self,robot0_act_pose,robot1_act_pose,robot2_act_pose):
        self.robot0_actual_pose_broadcaster.sendTransform((robot0_act_pose.position.x,robot0_act_pose.position.y,0), 
        (robot0_act_pose.orientation.x,robot0_act_pose.orientation.y,robot0_act_pose.orientation.z,robot0_act_pose.orientation.w),
        rospy.Time.now(), "robot0/actual_pose", "map")

        self.robot1_actual_pose_broadcaster.sendTransform((robot1_act_pose.position.x,robot1_act_pose.position.y,0), 
        (robot1_act_pose.orientation.x,robot1_act_pose.orientation.y,robot1_act_pose.orientation.z,robot1_act_pose.orientation.w),
        rospy.Time.now(), "robot1/actual_pose", "map")

        self.robot2_actual_pose_broadcaster.sendTransform((robot2_act_pose.position.x,robot2_act_pose.position.y,0), 
        (robot2_act_pose.orientation.x,robot2_act_pose.orientation.y,robot2_act_pose.orientation.z,robot2_act_pose.orientation.w),
        rospy.Time.now(), "robot2/actual_pose", "map")
       



    def config(self):
        self.control_rate = rospy.get_param("~control_rate")
        robot0_trajectory_topic = rospy.get_param("~robot0_trajectory_topic")
        robot1_trajectory_topic = rospy.get_param("~robot1_trajectory_topic")
        robot2_trajectory_topic = rospy.get_param("~robot2_trajectory_topic")
        rospy.Subscriber(robot0_trajectory_topic, Path, self.robot0_trajectory_cb)
        rospy.Subscriber(robot1_trajectory_topic, Path, self.robot1_trajectory_cb)
        rospy.Subscriber(robot2_trajectory_topic, Path, self.robot2_trajectory_cb)
        self.robot0_trajectory = Mypath()
        self.robot1_trajectory = Mypath()
        self.robot2_trajectory = Mypath()
        robot0_cmd_vel_topic = rospy.get_param("~robot0_cmd_vel_topic")
        robot1_cmd_vel_topic = rospy.get_param("~robot1_cmd_vel_topic")
        robot2_cmd_vel_topic = rospy.get_param("~robot2_cmd_vel_topic")
        self.robot0_twist_publisher = rospy.Publisher(robot0_cmd_vel_topic,Twist,queue_size=5)
        self.robot1_twist_publisher = rospy.Publisher(robot1_cmd_vel_topic,Twist,queue_size=5)
        self.robot2_twist_publisher = rospy.Publisher(robot2_cmd_vel_topic,Twist,queue_size=5)
        self.robot0_twist = Twist()
        self.robot1_twist = Twist()
        self.robot2_twist = Twist()
        self.robot0_trajectory_received = False
        self.robot1_trajectory_received = False
        self.robot2_trajectory_received = False
        self.robot0_act_pose = Pose()
        self.robot1_act_pose = Pose()
        self.robot2_act_pose = Pose()
        robot0_pose_topic = rospy.get_param("~robot0_pose_topic")
        robot1_pose_topic = rospy.get_param("~robot1_pose_topic")
        robot2_pose_topic = rospy.get_param("~robot2_pose_topic")
        rospy.Subscriber(robot0_pose_topic, Pose, self.robot0_pose_cb)
        rospy.Subscriber(robot1_pose_topic, Pose, self.robot1_pose_cb)
        rospy.Subscriber(robot2_pose_topic, Pose, self.robot2_pose_cb)
        self.robot0_target_pose_broadcaster = tf.TransformBroadcaster()
        self.robot1_target_pose_broadcaster = tf.TransformBroadcaster()
        self.robot2_target_pose_broadcaster = tf.TransformBroadcaster()
        self.robot0_actual_pose_broadcaster = tf.TransformBroadcaster()
        self.robot1_actual_pose_broadcaster = tf.TransformBroadcaster()
        self.robot2_actual_pose_broadcaster = tf.TransformBroadcaster()


if __name__=="__main__":
    exe = execute_trajectories_node()
    exe.run()

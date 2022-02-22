#! /usr/bin/env python3

import rospy
import tf
from mypath import MyTrajectory
from nav_msgs.msg import Path
import math
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Pose
from cartesian_controller import cartesian_controller



class execute_trajectories_node():

    def __init__(self):
        rospy.init_node("lyapunov_controller_node")
        rospy.loginfo("lyapunov controller running")
        self.config()
        




    def run(self):
        while len(self.target_trajectories) != self.number_of_robots and not rospy.is_shutdown():
            rospy.sleep(0.1)
                
        print("all trajecories received")
        rate = rospy.Rate(self.control_rate)
        idx = 0
        #u_v = [0 for element in range(self.number_of_robots)]
        #u_w = [0 for element in range(self.number_of_robots)]

        while not rospy.is_shutdown() and idx < len(self.target_trajectories[0].v):
         
            for i in range(0,self.number_of_robots):
                act_pose        = self.robot_poses[i]
                set_pose_x      = self.target_trajectories[i].x[idx]
                set_pose_y      = self.target_trajectories[i].y[idx]
                set_pose_phi    = self.target_trajectories[i].phi[idx]
                w_target        = self.target_trajectories[i].w[idx] * self.control_rate
                v_target        = self.target_trajectories[i].v[idx] * self.control_rate
                u_v, u_w = cartesian_controller(act_pose,set_pose_x,set_pose_y,w_target,v_target,set_pose_phi)

                self.robot_command.linear.x = u_v
                self.robot_command.angular.z = u_w
                self.cmd_vel_publishers[i].publish(self.robot_command)

                self.target_pose_broadcaster([set_pose_x,set_pose_y,set_pose_phi],i)
                self.actual_pose_broadcaster(act_pose,i)

            idx += 1
            rate.sleep()



    def trajectory_cb(self,Path,robot_index):
        trajectory = MyTrajectory()
        trajectory.x = []
        trajectory.y = []
        trajectory.phi = []
        path_len = len(Path.poses)
        for i in range(0,path_len-1):
            trajectory.x.append(Path.poses[i].pose.position.x)
            trajectory.y.append(Path.poses[i].pose.position.y)
            phi = math.atan2(Path.poses[i+1].pose.position.y-Path.poses[i].pose.position.y,Path.poses[i+1].pose.position.x-Path.poses[i].pose.position.x)
            trajectory.phi.append(phi)
        
        trajectory.v = [0.0]
        trajectory.w = [0.0]
        for i in range(1,path_len-2):
            trajectory.v.append(math.sqrt((trajectory.x[i+1]-trajectory.x[i])**2 + (trajectory.y[i+1]-trajectory.y[i])**2 ))
            trajectory.w.append(trajectory.phi[i+1]-trajectory.phi[i])

        self.target_trajectories.append(trajectory)
        rospy.loginfo("trajecory " + str(robot_index) +" received")


    def robot_pose_cb(self,Pose,robot_index):
        self.robot_poses[robot_index] = Pose.pose.pose


    def target_pose_broadcaster(self,target_pose,robot_id):
        frame_id = "robot" + str(robot_id) + "/target_pose"
        self.pose_broadcaster.sendTransform((target_pose[0], target_pose[1], 0),
                     tf.transformations.quaternion_from_euler(0, 0, target_pose[2]),
                     rospy.Time.now(), frame_id, "map")

    def actual_pose_broadcaster(self,actual_pose,robot_id):
            frame_id = "robot" + str(robot_id) + "/actual_pose"
            self.pose_broadcaster.sendTransform((actual_pose.position.x, actual_pose.position.y, 0),
                     (actual_pose.orientation.x,actual_pose.orientation.y,actual_pose.orientation.z,actual_pose.orientation.w),
                     rospy.Time.now(), frame_id, "map")

    def config(self):
        self.number_of_robots = rospy.get_param("~number_of_robots")
        self.control_rate = rospy.get_param("~control_rate")
        self.target_trajectories = []
        self.robot_poses = []
        self.cmd_vel_publishers = []
        self.robot_command = Twist()
        self.pose_broadcaster = tf.TransformBroadcaster()

        for i in range(0,self.number_of_robots):
            param = "~robot" + str(i) + "_trajectory_topic"
            robotX_trajectory_topic = rospy.get_param(param)
            rospy.Subscriber(robotX_trajectory_topic, Path, self.trajectory_cb, i)

            param = "~robot" + str(i) + "_pose_topic"
            robotX_pose_topic = rospy.get_param(param)
            self.robot_poses.append(Pose())
            rospy.Subscriber(robotX_pose_topic, PoseWithCovarianceStamped, self.robot_pose_cb, i)

            param = "~robot" + str(i) + "_cmd_vel_topic"
            topic = rospy.get_param(param)
            self.cmd_vel_publishers.append(rospy.Publisher(topic,Twist,queue_size=5))

       



if __name__=="__main__":
    exe = execute_trajectories_node()
    exe.run()
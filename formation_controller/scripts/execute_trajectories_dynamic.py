#! /usr/bin/env python3

import rospy
import tf
from tf import transformations
from mypath import MyTrajectory
from nav_msgs.msg import Path, Odometry
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

        # Move to initial pose
        # Turn towards the initial pose
        for i in range(0,self.initial_pose_iterations):
            correct_orientation = 0
            while not rospy.is_shutdown() and (correct_orientation < self.number_of_robots):
                for i in range(0,self.number_of_robots):
                    act_pose        = self.robot_poses[i]
                    set_pose_x      = self.target_trajectories[i].x[0]
                    set_pose_y      = self.target_trajectories[i].y[0]
                    phi_actual = transformations.euler_from_quaternion([act_pose.orientation.x,act_pose.orientation.y,act_pose.orientation.z,act_pose.orientation.w])
                    phi_target = math.atan2(set_pose_y-act_pose.position.y,set_pose_x-act_pose.position.x)
                    e_phi = phi_target - phi_actual[2]

                    self.robot_command.angular.z = self.K_phi * e_phi
                    if abs(self.robot_command.angular.z) > self.limit_w:
                        self.robot_command.angular.z = self.robot_command.angular.z / abs(self.robot_command.angular.z) * self.limit_w
                    if abs(e_phi) < self.target_threshhold_angular:
                        self.robot_command.angular.z = 0
                        correct_orientation += 1
                    self.cmd_vel_publishers[i].publish(self.robot_command)

                    print(i,"phi",e_phi,self.robot_command.angular.z)
                rate.sleep()

            # move linear to the initial pose
            correct_distance = 0
            while not rospy.is_shutdown() and correct_distance < self.number_of_robots:
                    for i in range(0,self.number_of_robots):
                        act_pose        = self.robot_poses[i]
                        set_pose_x      = self.target_trajectories[i].x[0]
                        set_pose_y      = self.target_trajectories[i].y[0]
                        e_d = math.sqrt((set_pose_x-act_pose.position.x)**2 + (set_pose_y-act_pose.position.y) **2 )

                        self.robot_command.linear.x = self.K_d * e_d
                        if abs(self.robot_command.linear.x) > self.limit_x:
                            self.robot_command.linear.x = self.robot_command.linear.x / abs(self.robot_command.linear.x) * self.limit_x
                        if abs(e_d) < self.target_threshhold_linear:
                            self.robot_command.linear.x = 0
                            correct_distance += 1
                        self.cmd_vel_publishers[i].publish(self.robot_command)
                    
                        print("l",e_d)
                        rate.sleep()
            self.target_threshhold_linear *= 0.5

        # set correct orientation
        correct_orientation = 0
        while not rospy.is_shutdown() and correct_orientation < self.number_of_robots:
                for i in range(0,self.number_of_robots) :
                    act_pose        = self.robot_poses[i]
                    phi_actual = transformations.euler_from_quaternion([act_pose.orientation.x,act_pose.orientation.y,act_pose.orientation.z,act_pose.orientation.w])
                    phi_target = self.target_trajectories[i].phi[0]
                    e_phi = phi_target - phi_actual[2]

                    self.robot_command.angular.z = self.K_phi * e_phi
                    if abs(self.robot_command.angular.z) > self.limit_w:
                        self.robot_command.angular.z = self.robot_command.angular.z / abs(self.robot_command.angular.z) * self.limit_w
                    if abs(e_phi) < self.target_threshhold_angular:
                        self.robot_command.angular.z = 0
                        correct_orientation += 1
                    self.cmd_vel_publishers[i].publish(self.robot_command)
                
                    print(e_phi)
            
            
              
        while not rospy.is_shutdown() and idx < len(self.target_trajectories[0].v):
         
            for i in range(0,self.number_of_robots):
                actual_pose     = self.robot_poses[i]

                target_pose     = Pose()
                target_pose.position.x = self.target_trajectories[i].x[idx]
                target_pose.position.y = self.target_trajectories[i].y[idx]
                q = transformations.quaternion_from_euler(0,0,self.target_trajectories[i].phi[idx])
                target_pose.orientation.x = q[0]
                target_pose.orientation.y = q[1]
                target_pose.orientation.z = q[2]
                target_pose.orientation.w = q[3]

                target_velocity = Twist()
                target_velocity.linear.x = self.target_trajectories[i].v[idx] * self.control_rate
                target_velocity.angular.z = self.target_trajectories[i].w[idx] * self.control_rate

                u_v, u_w = cartesian_controller(actual_pose,target_pose,target_velocity)
                rospy.loginfo_throttle(1, "u_v: %f, u_w: %f", u_v, u_w)
                rospy.loginfo_throttle(1, "target velocity= " + str(target_velocity.linear.x))

                self.robot_command.linear.x = u_v
                self.robot_command.angular.z = u_w
                self.cmd_vel_publishers[i].publish(self.robot_command)

                self.target_pose_broadcaster(target_pose,i)
                # self.actual_pose_broadcaster(actual_pose,i)

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
        self.actual_pose_broadcaster(Pose.pose.pose,robot_index)


    def target_pose_broadcaster(self,target_pose,robot_id):
        frame_id = "robot" + str(robot_id) + "/target_pose"
        self.pose_broadcaster.sendTransform((target_pose.position.x, target_pose.position.y, 0),
                     (target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w),
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
        self.K_phi = 0.5
        self.K_d = 0.5
        self.limit_w = 0.5
        self.limit_x = 0.2
        self.target_threshhold_angular = 0.02
        self.target_threshhold_linear = 1.0
        self.initial_pose_iterations = 3

        for i in range(0,self.number_of_robots):
            param = "~robot" + str(i) + "_trajectory_topic"
            robotX_trajectory_topic = rospy.get_param(param)
            rospy.Subscriber(robotX_trajectory_topic, Path, self.trajectory_cb, i)

            param = "~robot" + str(i) + "_pose_topic"
            robotX_pose_topic = rospy.get_param(param)
            self.robot_poses.append(Pose())
            rospy.Subscriber(robotX_pose_topic, Odometry, self.robot_pose_cb, i) #PoseWithCovarianceStamped

            param = "~robot" + str(i) + "_cmd_vel_topic"
            topic = rospy.get_param(param)
            self.cmd_vel_publishers.append(rospy.Publisher(topic,Twist,queue_size=5))

       



if __name__=="__main__":
    exe = execute_trajectories_node()
    exe.run()

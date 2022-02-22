#! /usr/bin/env python3

import rospy
import tf
from mypath import MyTrajectory
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
        while len(self.target_trajectories) != self.number_of_robots and not rospy.is_shutdown():
            rospy.sleep(0.1)
                
        print("all trajecories received")
        rate = rospy.Rate(self.control_rate)
        idx = 0
        #u_v = [0 for element in range(self.number_of_robots)]
        #u_w = [0 for element in range(self.number_of_robots)]

        while not rospy.is_shutdown() and idx < len(self.target_trajectories[0].v):
         
            for i in range(0,self.number_of_robots):
                print(self.target_trajectories[i].x[idx])
                act_pose        = self.robot_poses[i]
                set_pose_x      = self.target_trajectories[i].x[idx]
                set_pose_y      = self.target_trajectories[i].y[idx]
                set_pose_phi      = self.target_trajectories[i].phi[idx]
                w_target        = self.target_trajectories[i].w[idx]
                v_target        = self.target_trajectories[i].v[idx]
                u_v, u_w = cartesian_controller(act_pose,set_pose_x,set_pose_y,w_target,v_target,set_pose_phi)

                self.robot_command.linear.x = u_v
                self.robot_command.angular.z = u_w
                self.cmd_vel_publishers[i].publish(self.robot_command)

                self.target_pose_broadcaster([set_pose_x,set_pose_y,set_pose_phi],i)
            
            # self.target_pose_broadcaster([self.robot0_trajectory.x[idx],self.robot0_trajectory.y[idx],self.robot0_trajectory.phi[idx]], \
            #     [self.robot1_trajectory.x[idx],self.robot1_trajectory.y[idx],self.robot1_trajectory.phi[idx]], \
            #     [self.robot2_trajectory.x[idx],self.robot2_trajectory.y[idx],self.robot2_trajectory.phi[idx]])

            # self.actual_pose_broadcaster(self.robot0_act_pose,self.robot1_act_pose,self.robot2_act_pose)

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
        print("trajecory " + str(robot_index) +" received")


    def robot_pose_cb(self,Pose,robot_index):
        self.robot_poses[robot_index] = Pose.pose.pose


    def target_pose_broadcaster(self,target_pose,robot_id):
        frame_id = "robot" + str(robot_id) + "/target_pose"
        self.pose_broadcaster.sendTransform((target_pose[0], target_pose[1], 0),
                     tf.transformations.quaternion_from_euler(0, 0, target_pose[2]),
                     rospy.Time.now(), frame_id, "map")


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
        self.number_of_robots = rospy.get_param("~number_of_robots")
        self.control_rate = rospy.get_param("~control_rate")
        self.target_trajectories = []
        self.robot_poses = []
        self.cmd_vel_publishers = []
        self.robot_command = Twist()

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

        self.robot0_trajectory = MyTrajectory()
        self.robot1_trajectory = MyTrajectory()
        self.robot2_trajectory = MyTrajectory()
        # robot0_cmd_vel_topic = rospy.get_param("~robot0_cmd_vel_topic")
        # robot1_cmd_vel_topic = rospy.get_param("~robot1_cmd_vel_topic")
        # robot2_cmd_vel_topic = rospy.get_param("~robot2_cmd_vel_topic")
        # self.robot0_twist_publisher = rospy.Publisher(robot0_cmd_vel_topic,Twist,queue_size=5)
        # self.robot1_twist_publisher = rospy.Publisher(robot1_cmd_vel_topic,Twist,queue_size=5)
        # self.robot2_twist_publisher = rospy.Publisher(robot2_cmd_vel_topic,Twist,queue_size=5)
        # self.robot0_twist = Twist()
        # self.robot1_twist = Twist()
        # self.robot2_twist = Twist()
        self.robot0_trajectory_received = False
        self.robot1_trajectory_received = False
        self.robot2_trajectory_received = False
        self.robot0_act_pose = Pose()
        self.robot1_act_pose = Pose()
        self.robot2_act_pose = Pose()
        # robot0_pose_topic = rospy.get_param("~robot0_pose_topic")
        # robot1_pose_topic = rospy.get_param("~robot1_pose_topic")
        # robot2_pose_topic = rospy.get_param("~robot2_pose_topic")
        # rospy.Subscriber(robot0_pose_topic, Pose, self.robot0_pose_cb)
        # rospy.Subscriber(robot1_pose_topic, Pose, self.robot1_pose_cb)
        # rospy.Subscriber(robot2_pose_topic, Pose, self.robot2_pose_cb)
        self.pose_broadcaster = tf.TransformBroadcaster()
        self.robot0_target_pose_broadcaster = tf.TransformBroadcaster()
        self.robot1_target_pose_broadcaster = tf.TransformBroadcaster()
        self.robot2_target_pose_broadcaster = tf.TransformBroadcaster()
        self.robot0_actual_pose_broadcaster = tf.TransformBroadcaster()
        self.robot1_actual_pose_broadcaster = tf.TransformBroadcaster()
        self.robot2_actual_pose_broadcaster = tf.TransformBroadcaster()


if __name__=="__main__":
    exe = execute_trajectories_node()
    exe.run()

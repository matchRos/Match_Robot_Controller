#!/usr/bin/env python
import rospy
import numpy
from my_panda_controllers.msg import MyVelocity
from geometry_msgs.msg import Twist, Pose
from franka_msgs.msg import FrankaState


class next_layer():

    def __init__(self):
        rospy.init_node('layer')
        from franka_msgs.msg import FrankaState
        self.target_velocity = MyVelocity()
        self.pose = Pose()
        self.target_height = 0.0
        self.initial_run = True
        self.goal_tolerance = 0.001
        self.cmd_vel = Twist()
        self.K_p = 1
        self.layer_height = -3
        
        
        self.pub = rospy.Publisher("cartesian_EE_velocity", Twist, queue_size=1)
        rospy.Subscriber("franka_state_controller/franka_states",FrankaState, self.pose_cb) 
        rospy.sleep(1)
        self.run()  
        rospy.spin()
        
        
    def run(self):

        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and abs(self.pose.position.z-self.target_height) > self.goal_tolerance:
            e = self.target_height - self.pose.position.z
            self.cmd_vel.linear.z = self.K_p * e
            print(self.cmd_vel.linear.z)
            self.pub.publish(self.cmd_vel)
            rate.sleep()
            
        self.cmd_vel.linear.z = 0.0
        print("done")
        self.pub.publish(self.cmd_vel)
            
    def pose_cb(self,data): 
        self.pose.position.x = data.O_T_EE[12]
        self.pose.position.y = data.O_T_EE[13]
        self.pose.position.z = data.O_T_EE[14]

        if self.initial_run == True:
            self.initial_run = False
            self.target_height = self.pose.position.z + self.layer_height * 0.001
            


if __name__ == '__main__':
    try:
        next_layer()
    except rospy.ROSInterruptException:
        pass

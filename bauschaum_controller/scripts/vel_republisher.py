#!/usr/bin/env python
import rospy
from my_panda_controllers.msg import MyVelocity
from geometry_msgs.msg import Twist, Pose
from franka_msgs.msg import FrankaState

class auswertung():

    def __init__(self):
        rospy.init_node('auswertung')
        self.config()
        self.target_velocity = MyVelocity()
        self.pose = Pose()
        self.initial_pose = Pose()
        self.target_velocity.vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.initial_run = True
        
        self.pub_vel = rospy.Publisher('my_cartesian_velocity_controller/my_velocities', MyVelocity, queue_size=2)    
        self.sub = rospy.Subscriber("cartesian_EE_velocity", Twist, self.subcb)
        rospy.Subscriber(self.tf_prefix+"franka_state_controller/franka_states",FrankaState, self.pose_cb)   
        rospy.spin()
        
    def config(self):
        self.vel_limit_x = rospy.get_param('~vel_limit_x')
        self.vel_limit_y = rospy.get_param('~vel_limit_y')
        self.tf_prefix = rospy.get_param('~tf_prefix')
        rospy.loginfo("config loaded")

    def subcb(self,data): # type: (Twist) -> Pose
        v_x = data.linear.x
        v_y = data.linear.y
        
        if v_x > 0 and v_x < self.vel_limit_x:
            self.target_velocity.vel[0]=v_x
        elif v_x < 0 and v_x > -self.vel_limit_x:
            self.target_velocity.vel[0]=v_x
        elif v_x > 0 and v_x > self.vel_limit_x:
            self.target_velocity.vel[0] = self.vel_limit_x
        elif v_x < 0 and v_x < self.vel_limit_x:
            self.target_velocity.vel[0] = -self.vel_limit_x
            
        if v_y > 0 and v_y < self.vel_limit_y:
            self.target_velocity.vel[1]=v_y
        elif v_y < 0 and v_y > -self.vel_limit_y:
            self.target_velocity.vel[1]=v_y
        elif v_y > 0 and v_y > self.vel_limit_y:
            self.target_velocity.vel[1] = self.vel_limit_y
        elif v_y < 0 and v_y < self.vel_limit_x:
            self.target_velocity.vel[1] = -self.vel_limit_y

        print(self.target_velocity)
        
    
    def pose_cb(self,data):
        self.pose.position.x = data.O_T_EE[12]
        self.pose.position.Y = data.O_T_EE[13]
        self.pose.position.Z = data.O_T_EE[14]

        if self.initial_run == True:
            self.initial_run = False
            self.initial_pose = self.pose

if __name__ == '__main__':
    try:
        auswertung()
    except rospy.ROSInterruptException:
        pass
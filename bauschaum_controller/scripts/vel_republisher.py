#!/usr/bin/env python
from numpy.lib.financial import rate
import rospy
import numpy
from my_panda_controllers.msg import MyVelocity
from geometry_msgs.msg import Twist, Pose
from franka_msgs.msg import FrankaState

class auswertung():

    def __init__(self):
        rospy.init_node('auswertung')
        self.config()
        self.target_velocity = MyVelocity()
        self.mir_target_velocity = Twist()
        self.pose = Pose()
        self.initial_pose = Pose()
        self.target_velocity.vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.initial_run = True
        self.running_average_mir = 0.0 
        self.running_average_y = 0.0 
        self.running_average_z = 0.0 
        self.vel_y_old = 0.0
        self.vel_z_old = 0.0
        self.v_mir = 0.0
        self.v_y = 0.0
        self.v_z = 0.0
        
        
        self.pub_EE_vel = rospy.Publisher('my_cartesian_velocity_controller/my_velocities', MyVelocity, queue_size=2) 
        self.pub_MiR_vel = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=1)   
        self.sub = rospy.Subscriber("cartesian_EE_velocity", Twist, self.subcb)
        rospy.Subscriber(self.tf_prefix+"franka_state_controller/franka_states",FrankaState, self.pose_cb) 
        rospy.sleep(1)
        self.run()  
        rospy.spin()
        
    def config(self):
        self.vel_limit_x = rospy.get_param('~vel_limit_x')
        self.vel_limit_y = rospy.get_param('~vel_limit_y')
        self.vel_limit_z = rospy.get_param('~vel_limit_z')
        self.acc_limit_y = rospy.get_param('~acc_limit_y')
        self.acc_limit_z = rospy.get_param('~acc_limit_z')
        self.vel_limit_mir = rospy.get_param('~vel_limit_mir')
        self.vel_reference_mir = rospy.get_param('~vel_reference_mir')
        self.tf_prefix = rospy.get_param('~tf_prefix')
        
        rospy.loginfo("config loaded")

    def subcb(self,data): # type: (Twist) -> Pose
            self.v_mir = self.vel_reference_mir * data.linear.x
            self.v_y = data.linear.y
            self.v_z = data.linear.z


    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            
            v_mir = self.v_mir
            v_y = self.v_y
            v_z = self.v_z
            
            if v_mir >= 0.0 and v_mir < self.vel_limit_mir:
                vel_x=v_mir
            elif v_mir < 0.0 and v_mir > -self.vel_limit_mir:
                vel_x=v_mir
            elif v_mir > 0.0 and v_mir >= self.vel_limit_mir:
                vel_x = self.vel_limit_mir
            elif v_mir < 0.0 and v_mir <= self.vel_limit_mir:
                vel_x = -self.vel_limit_mir
                
            
            if v_y >= 0.0 and v_y < self.vel_limit_y:
                vel_y=v_y
            elif v_y < 0.0 and v_y > -self.vel_limit_y:
                vel_y=v_y
            elif v_y > 0.0 and v_y >= self.vel_limit_y:
                vel_y = self.vel_limit_y
            elif v_y < 0.0 and v_y <= self.vel_limit_y:
                vel_y = -self.vel_limit_y
                
            if abs(v_z) > self.vel_limit_z:
                vel_z = numpy.sign(v_z)*self.vel_limit_z
            else:   
                vel_z = v_z
                
            print(vel_z)
            
            if abs(vel_y-self.vel_y_old) > self.acc_limit_y:
                vel_y = vel_y + numpy.sign(vel_y) * self.acc_limit_y
                print("Y acc limit reached:", vel_y, numpy.sign(vel_y))
                
            
            if abs(vel_z-self.vel_z_old) > self.acc_limit_z:
                vel_z = vel_z + numpy.sign(vel_z) * self.acc_limit_z
                print("Z acc limit reached:", vel_z, numpy.sign(vel_z))
        
                
            self.vel_y_old = vel_y
            self.vel_z_old = vel_z
                
            self.running_average_mir = self.running_average_mir * 0.90 + vel_x * 0.1
            self.running_average_y = self.running_average_y * 0.90 + vel_y * 0.1
            self.running_average_z = self.running_average_z * 0.90 + vel_z * 0.1
            
            if vel_z == 0.0:
                self.running_average_z = 0.0
            
            #self.target_velocity.vel[0] = self.running_average_mir 
            self.target_velocity.vel[1] = self.running_average_y    
            self.target_velocity.vel[2] = self.running_average_z 
            
            self.mir_target_velocity.linear.x = self.running_average_mir 
                
                #rospy.loginfo_throttle(1,"MiR:", self.mir_target_velocity.linear.x, "Panda: ", self.target_velocity.vel[1])

            self.pub_EE_vel.publish(self.target_velocity)
            self.pub_MiR_vel.publish(self.mir_target_velocity)
            rate.sleep()
        
    
    def pose_cb(self,data): 
        self.pose.position.x = data.O_T_EE[12]
        self.pose.position.y = data.O_T_EE[13]
        self.pose.position.z = data.O_T_EE[14]

        if self.initial_run == True:
            self.initial_run = False
            self.initial_pose = self.pose

if __name__ == '__main__':
    try:
        auswertung()
    except rospy.ROSInterruptException:
        pass
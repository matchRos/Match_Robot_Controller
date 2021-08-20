#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
from geometry_msgs.msg import PoseArray, Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
import numpy
import math

class auswertung():

    def __init__(self):
        rospy.init_node('auswertung')
        self.poseArray = PoseArray() # Type: PoseArray
        self.pose = Pose()
        self.laser_projector = LaserProjection()
        #self.sub = rospy.Subscriber("cloud_out", PointCloud2, self.subcb)
        self.sub = rospy.Subscriber("ranges", Float64MultiArray, self.subcb)
        self.vel_pub = rospy.Publisher("cartesian_EE_velocity",Twist, queue_size=1)
        self.cmd_vel = Twist()
        rospy.spin()
        #self.pub = rospy.Publisher('ranges', JointState, queue_size=10)
        self.one_step_before = 0.0
        self.delta_before = 0.0

    def subcb(self,ranges):

        #convert scanner data to list
        data = []
        for i in ranges.data:
            #delete inf values
            if i > 999:
                data.append(-999)
            else:
                data.append(i)


        #CALCULATION OF ROBOTSPEEDCORRECTION (sideways)

            #calculate max as mark for the middle
        maximum_height = max(data)
            #use index of max to calculate mid-offset
        index_of_max = data.index(max(data))
        if maximum_height <= -800:
            index_of_max = 400
        delta_to_mid = index_of_max - 435
            #transform mid-offset to speed correction
        kp_side = 0.0001
        kd_side = 0.000001
        now = rospy.get_rostime()
        speed_correction_sideways = kp_side * delta_to_mid  #+ kd_side * (delta_to_mid - self.delta_before) / self.one_step_before
            #publish
        self.cmd_vel.linear.y = -speed_correction_sideways
            #printing
        rospy.loginfo_throttle(0.5,speed_correction_sideways)
        

        #CALCULATION OF PLATFORMSPEEDCORRECTION (forward)

            #targeted height value as distance between printing surface and scanner zero level
        height_target = -0.01913
            #heigth difference
        center_of_dataset = []
        for i in range(1,len(data)):
            if i >= index_of_max-25 and i <= index_of_max+25:
                center_of_dataset.append(data[i])
        median_of_center_of_dataset = numpy.median(center_of_dataset)
        if median_of_center_of_dataset <= -800:
            median_of_center_of_dataset = 0.025
        delta_height = height_target - median_of_center_of_dataset
            #transform heigth-difference to feedrate correction
        k_forward = 2.0
        speed_correction_forward =1 - k_forward*delta_height
            #publish
        self.cmd_vel.linear.x = speed_correction_forward
            #printing
        rospy.loginfo_throttle(0.5,speed_correction_forward)

        #rospy.loginfo("Got scan, projecting")
        # cloud_points = list(pc2.read_points(cloud, skip_nans=True, field_names = ("x", "y", "z")))
        # print("running")
        # maximum_height = max(cloud_points)
        # #index_of_max = maximum_height.index(cloud_points)
        
        # points = []
        # for i in range(1,len(cloud_points)):
        #     #print(i[2])
        #     if cloud_points[i][2] > 9999:
        #         points.append(0)
        #     else:
        #         points.append(cloud_points[i][2])

        # maximum_height2 = max(points)
        # rospy.loginfo(maximum_height2)
        # #cloud = self.laser_projector.projectLaser(scan)
        #gen = pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z"))
        #self.xyz_generator = gen
        #)

        #self.cmd_vel.linear.z = index_of_max
        self.vel_pub.publish(self.cmd_vel)

if __name__ == '__main__':
    try:
        auswertung()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
import rospy 
import numpy as np
from dead_reckoning_class import dead_reckoning
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32MultiArray
import tf2_ros #ROS package to work with transformations 
from geometry_msgs.msg import TransformStamped

class localisation:
    def __init__(self , mode):
        ###--- Inicio del Nodo ---###
        rospy.init_node('localisation')
        rospy.on_shutdown(self.cleanup)

        wr_sub = "/wr"
        wl_sub = "/wl"
        self.ref = "/base_link"

        ###--- Subscriptores ---###
        rospy.Subscriber(wr_sub , Float32 , self.wr_cb)
        rospy.Subscriber(wl_sub , Float32 , self.wl_cb)
        rospy.Subscriber("/aruco_topic" , Float32MultiArray , self.ary_cb)
        rospy.Subscriber("/arucos_flag" , Bool , self.flag_cb)

        ###--- Publishers ---###
        self.odom_pub = rospy.Publisher("odom" , Odometry , queue_size=1)

        ###--- Robot Constants ---###
        self.r = 0.05
        self.l = 0.19
        self.dt = 0.02

        ###--- Variables ---####
        self.v = 0.0
        self.w = 0.0
        self.wr = 0.0 
        self.wl = 0.0
        self.ar_arr = []
        self.flag = False
        
        
        self.odom = Odometry()
        self.covariance = dead_reckoning(self.dt , 0.25 , 0.5 , 0)
        rate = rospy.Rate(int(1.0/self.dt))
        self.tf2_ros = tf2_ros.TransformBroadcaster() # Create a TransformBroadcaster object
        self.t = TransformStamped()

        #while rospy.get_time() == 0: print ("Simulacion no iniciada")#Descomentar en simulacion 

        while not rospy.is_shutdown():
            self.get_robot_velocities()
            cov_mat , u = self.covariance.calculate(self.v , self.w , self.wr , self.wl , self.flag , self.ar_arr)
            self.get_transform(u)
            self.get_odom(cov_mat , u)
            print(u)

            ###--- Publish ---###
            self.odom_pub.publish(self.odom)
            rate.sleep() 

    def get_robot_velocities (self):
        self.v = (self.r * (self.wr + self.wl))/2
        self.w = self.r * ((((2*self.v/self.r) - self.wl)-self.wl)/self.l)

    def get_odom (self , cov_mat , u): 
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = self.ref
        self.odom.pose.pose.position.x = u[0] #+ 0.1
        self.odom.pose.pose.position.y = u[1]

        quat = quaternion_from_euler(0 , 0 , u[2])
        self.odom.pose.pose.orientation.x = quat[0]
        self.odom.pose.pose.orientation.y = quat[1]
        self.odom.pose.pose.orientation.z = quat[2]
        self.odom.pose.pose.orientation.w = quat[3]

        self.odom.pose.covariance = [0.0] * 36

        self.odom.pose.covariance[0] = cov_mat[0][0] * 17 #Covariance in x
        self.odom.pose.covariance[1] = cov_mat[0][1] #Covariance in xy 
        self.odom.pose.covariance[5] = cov_mat[0][2] #Covariance in x theta
        self.odom.pose.covariance[6] = cov_mat[1][0] #Covariance in y x
        self.odom.pose.covariance[7] = cov_mat[1][1] #Covariance in y 
        self.odom.pose.covariance[11] = cov_mat[1][2] #Covariance in y theta
        self.odom.pose.covariance[30] = cov_mat[2][0] #Covariance in theta x
        self.odom.pose.covariance[31] = cov_mat[2][1] #Covariance in theta y
        self.odom.pose.covariance[35] = cov_mat[2][2] * 5 #Covariance in theta

    def get_transform(self,u):
            # Fill the transformation information 
            self.t.header.stamp = rospy.Time.now() 
            self.t.header.frame_id = "odom" 
            self.t.child_frame_id = self.ref 
            self.t.transform.translation.x = u[0]
            self.t.transform.translation.y = u[1] 
            self.t.transform.translation.z = 0.0 
            # self.theta += 0.01 #theta will be increasing 
            #Clip the value of theta to the interval [-pi to pi] 
            theta = np.arctan2(np.sin(u[2]), np.cos(u[2]))
            #The transformation requires the orientation as a quaternion 
            q = quaternion_from_euler(0, 0, theta) 
            self.t.transform.rotation.x = q[0] 
            self.t.transform.rotation.y = q[1] 
            self.t.transform.rotation.z = q[2] 
            self.t.transform.rotation.w = q[3] 
            # A transformation is broadcasted instead of published 
            self.tf2_ros.sendTransform(self.t) #broadcast the transformation 
 
        
    def wr_cb (self , msg):
        self.wr = msg.data

    def wl_cb (self , msg):
        self.wl = msg.data

    def ary_cb (self , msg):
        self.ar_arr = msg.data

    def flag_cb (self , msg):
        self.flag = msg.data

    def cleanup (self):
        print ("Apagando Localsation")
        self.odom_pub.publish(Odometry())
        
        

if __name__ == "__main__": 
    localisation("sim")

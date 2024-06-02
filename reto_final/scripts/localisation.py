#!/usr/bin/env python

import rospy  
from std_msgs.msg import Float32, Int32
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32MultiArray 
import numpy as np 

class Localisation():  
    def _init_(self):
        print("entra al init")
        rospy.init_node('localisation') 
        
        rospy.Subscriber("wl", Float32, self.wl_cb) 
        rospy.Subscriber("wr", Float32, self.wr_cb) 
        rospy.Subscriber("aruco_topic", Float32MultiArray, self.aruco_cb)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1) 

        self.r = 0.05  
        self.L = 0.19  
        self.dt = 0.1
        
        self.x_aruco = 0
        self.y_aruco  = 0

        self.w = 0.0  
        self.v = 0.0  
        self.x = 0.4  
        self.y = 0.4  
        self.theta = 0.0  
        self.wr = 0.0  
        self.wl = 0.0
        self.id_aruco = 0
        
        self.Z = np.array([[0,0],[0,0]])
        
        self.I = np.eye(3)
        #self.I = np.array([[1,0,0],[0,1,0],[0,0,1]])  
        
        self.z_covariance = np.zeros((3, 3))
        #self.z_covariance = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
        self.miu = np.array([0.4, 0.4, 0.0])
        self.wr_k = 0.15
        self.wl_k = 0.025
        self.w_sigma_const = 0.5 * self.dt * self.r
        self.get_aruco = False
        
        rate = rospy.Rate(int(1.0/self.dt))

        while not rospy.is_shutdown():
            
            [self.v, self.w] = self.get_robot_vel(self.wr, self.wl)
            self.prediccion()
            
            if self.get_aruco == True:
                self.correction_step()
                self.get_aruco = False

            self.update_robot_pose(self.v, self.w) 
            
            odom_msg = self.get_odom_stamped(self.x, self.y, self.theta, self.z_covariance, self.v, self.w) 
            self.odom_pub.publish(odom_msg) 

            rate.sleep() 

    def wl_cb(self, msg): 
        self.wl = msg.data

    def wr_cb(self, msg): 
        self.wr = msg.data       

    def get_robot_vel(self, wr, wl): 
        v = ((wl + wr) / 2) * self.r 
        w = ((wr - wl) / self.L) * self.r
        return [v, w] 
    
    def aruco_cb(self, msg):
        id_aruco = msg.data[0]
        self.d_aruco = msg.data[1]
        self.theta_aruco = msg.data[2]
        
        self.id_arucos(id_aruco)
        self.get_aruco = True

    def id_arucos(self, id):
        x_y = {702: [0, 0.80], 701: [0, 1.60], 703: [1.73, 0.80],
               704: [2.63, 0.39], 705: [2.85, 0], 706: [2.865,2.0],
               707: [1.735, 1.22]}
        
        self.x_aruco = x_y[id][0]
        self.y_aruco = x_y[id][1]

    def get_odom_stamped(self, x, y, yaw, sigma, mu_v, mu_w): 
        odom_stamped = Odometry() 
        odom_stamped.header.frame_id = "odom" 
        odom_stamped.child_frame_id = "base_link"
        odom_stamped.header.stamp = rospy.Time.now() 
        odom_stamped.pose.pose.position.x = x
        odom_stamped.pose.pose.position.y = y

        quat = quaternion_from_euler(0, 0, yaw) 
        odom_stamped.pose.pose.orientation.x = quat[0]
        odom_stamped.pose.pose.orientation.y = quat[1]
        odom_stamped.pose.pose.orientation.z = quat[2]
        odom_stamped.pose.pose.orientation.w = quat[3]

        odom_array = np.array([[sigma[0][0], sigma[0][1], 0, 0, 0, sigma[0][2]],
                               [sigma[1][0], sigma[1][1], 0, 0, 0, sigma[1][2]],
                               [0, 0, 0, 0, 0, 0],
                               [0, 0, 0, 0, 0, 0],
                               [0, 0, 0, 0, 0, 0],
                               [sigma[2][0], sigma[2][1], 0, 0, 0, sigma[2][2]]])
        
        odom_stamped.pose.covariance = odom_array.flatten().tolist()

        odom_stamped.twist.twist.linear.x = mu_v
        odom_stamped.twist.twist.angular.z = mu_w
        
        return odom_stamped 

    def update_robot_pose(self, v, w): 
        self.x = self.x + v * np.cos(self.theta) * self.dt
        self.y = self.y + v * np.sin(self.theta) * self.dt
        self.theta = self.theta + w * self.dt

    def prediccion(self):
        self.k_d = 3.0
        self.k_t = 2.0
        self.sigma_k = np.array([[self.k_d * 2.8774e-07, 0],
                                [0, self.k_t * 7.3658e-09]])

        self.w_sigma = self.w_sigma_const * np.array([
            [np.cos(self.miu[2]), -np.sin(self.miu[2])],
            [np.sin(self.miu[2]), np.cos(self.miu[2])],
            [1 / self.L, -1 / self.L]])


        self.Q_k = self.w_sigma.dot(self.sigma_k).dot(self.w_sigma.T)

        self.H = np.array([[1, 0, -self.r * self.dt / 2 * (self.wr + self.wl) * np.sin(self.miu[2])],
            [0,1, self.r * self.dt / 2 * (self.wr + self.wl) * np.cos(self.miu[2])],
            [0,0,1]])

        self.miu = np.array([
            self.miu[0] + self.r * self.dt / 2 * (self.wr + self.wl) * np.cos(self.miu[2]),
            self.miu[1] + self.r * self.dt / 2 * (self.wr + self.wl) * np.sin(self.miu[2]),
            self.miu[2] + self.r * self.dt / self.L * (self.wr - self.wl)
        ])

        self.z_covariance = self.H.dot(self.z_covariance).dot(self.H.T) + self.Q_k

    def correction_step(self):
        self.delta_x = self.x_aruco - self.miu[0]
        self.delta_y = self.y_aruco - self.miu[1]
        self.p = self.delta_x * 2 + self.delta_y * 2
        self.z = np.array([self.d_aruco,self.theta_aruco])
        self.z_hat = np.array([np.sqrt(self.p), np.arctan2(self.delta_y, self.delta_x) - self.miu[2]])
        self.G = np.array([[-(self.delta_x / np.sqrt(self.p)), -(self.delta_y / np.sqrt(self.p)), 0], [(self.delta_y / self.p), -(self.delta_x / self.p), -1]])
        self.R_k = np.array([[0.01, 0],
                            [0, 0.02]])
        self.Z = self.G.dot(self.z_covariance).dot(self.G.T) + self.R_k
        self.K = self.z_covariance.dot(self.G.T).dot(np.linalg.inv(self.Z))

        self.miu = self.miu + self.K.dot(self.z - self.z_hat)
        self.z_covariance = (self.I - self.K.dot(self.G)).dot(self.z_covariance)
        print("pos " + str(self.miu))
        print("cov " + str(self.z_covariance))
        
if __name__ == "__main__": 
    Localisation()
   
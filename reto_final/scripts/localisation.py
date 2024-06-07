#!/usr/bin/env python3

import rospy  
from std_msgs.msg import Float32, Float32MultiArray, Bool
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, TransformStamped
import numpy as np 
from numpy.linalg import inv
import tf2_ros

class Localisation():  
    def __init__(self):
        rospy.init_node('localisation') 
        
        ################# SUBSCRIBERS ####################
        rospy.Subscriber("puzzlebot_1/wl", Float32, self.wl_cb) 
        rospy.Subscriber("puzzlebot_1/wr", Float32, self.wr_cb) 
        rospy.Subscriber("aruco_topic", Float32MultiArray, self.aruco_cb)
        rospy.Subscriber("goal_reached", Bool, self.goal_cb)

        ################# PUBLISHERS ######################
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)
        self.goal_pub = rospy.Publisher('goalmarker', Float32MultiArray, queue_size=1)

        ################# ROBOT CONSTANTS #################
        self.r = 0.05  
        self.L = 0.19
        self.dt = 0.05

        ############### INITIAL POSITION #################
        self.x = 2.3
        self.y = 1.04
        self.theta = np.pi
        
        ################ ARUCO IDENTIFIERS ###############
        self.x_aruco = 0
        self.y_aruco  = 0
        self.id_aruco = 0
        self.get_aruco = False  #Initially no aruco detected


        ######### LINEAR AND ANGULAR VELOCITY VARIABLES #########
        self.w = 0.0  
        self.v = 0.0  
        self.wr = 0.0  
        self.wl = 0.0
        self.flag = False

        self.first = True

        ############### EKF VARIABLES ##################

        ############## INITIAL CONDITIONS #############
        self.miu = np.array([self.x, self.y, self.theta]) #robot initial position [0] = X [1] = Y [2] = yaw Real position of the robot
        self.miu_hat = np.array([0, 0, 0]) #Estimated position

        self.sigma = np.zeros((3,3)) #initial covariance matrix Real covariance
        self.sigma_hat = np.zeros((3,3)) #Estimated covariance

        self.Z = np.zeros((2,2)) #Real position of the robot in respect to the aruco
        self.z_hat = np.zeros((2,2)) #Estimated position of the robot in respect to the aruco


        self.H = np.zeros((3,3)) #Linearized model for uncertainty propagation   
        self.G = np.zeros((2,3))
        self.gradient_W = np.zeros((3,2))

        self.Q = np.zeros((3,3))
        self.R = np.array([[0.02, 0.0], [0.0, 0.004]]) #2x2
        #self.gradient_W = np.zeros((3, 2))
        self.K = np.array((3, 2))
        self.covariance = np.zeros((2,3))

        ################# GAINS #################
        self.kl = 0.3
        self.kr = 0.2


        ################ GOALS ###################
        self.goals = [
            [0.6, 0.5],  # GOAL 1
            [1.38, 1.03], # GOAL 2
            [2.05, 0.35], # GOAL 3
            [2.97, 0.4],  # GOAL 4
            [0.6, 1.4]   # GOAL 5
        ]

        self.current_goal_index = 0

        rate = rospy.Rate(int(1.0/self.dt))

        self.goal = Float32MultiArray()

        while not rospy.is_shutdown():

            print("========================================")
            print("X del robot : " + str(self.x))
            print("Y del robot : " + str(self.y))
            print("Theta del robot : " + str(self.theta))
            print("\n")

            #self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))
        
            #Get linear and angular speeds
            self.v = ((self.wr + self.wl) / 2) * self.r 
            self.w = ((self.wr - self.wl) / self.L) * self.r

            #Calculate the covariance matrix
            self.covariance = np.array([[self.kl * abs(self.wr), 0], 
                                       [0, self.kr * abs(self.wl)]])

            #Calculate the jacobian matrix
            self.gradient_W = (0.5 * self.r * self.dt) * np.array([[np.cos(self.theta), np.cos(self.theta)],
                                                                  [np.sin(self.theta), np.sin(self.theta)],
                                                                  [2.0 / self.L, -2.0 / self.L]])
            
            #Calculate the noise covariance
            self.Q = self.gradient_W.dot(self.covariance).dot(self.gradient_W.T)

            #Call prediction step
            self.prediction()

            #Verify aruco detection
            if self.get_aruco == True:
                #If true call correction step
                self.correction()
                self.get_aruco = False
            else:
                #Propagate backwards miu and covariance for the odometry
                self.propagate()
            
            #Update odometry 
            odom_msg = self.get_odom_stamped()

            #Publish odometry
            self.odom_pub.publish(odom_msg)
            
            #Send transform from odom frame to base_link frame
            self.send_transform(odom_msg)

            #If bug0 returns that it is at the goal, it publishes a flag and we read it here to send the next goal
            if self.first == True:
                self.goal.data = self.goals[self.current_goal_index]
                self.current_goal_index += 1
                self.first = False

            if self.flag == True:
                if self.current_goal_index >= len(self.goals):
                    rospy.loginfo("!!!!!!!!All goals reached!!!!!!!!!!!")

                rospy.loginfo("Target # " + str(self.current_goal_index) + "at coord: " + str(self.goals[self.current_goal_index]))
                rospy.loginfo("Wait 5 seconds till next point is published")

                start_time = rospy.get_time()
                end_time = start_time + 5.0
                while rospy.get_time() < end_time:
                    elapsed_time = rospy.get_time() - start_time
                    percentage = (elapsed_time / 5.0) * 100
                    rospy.loginfo("Percentage: {:.2f}%".format(percentage))
                    rospy.sleep(0.1)
                    self.goal.data = self.goals[self.current_goal_index]
                rospy.loginfo("Sending new objective...")
                self.current_goal_index += 1
                self.flag = False

            self.goal_pub.publish(self.goal)
            rate.sleep() 

    
    ######################### EXTENDED KALMAN FILTER ########################

    def prediction(self):
        ############## CALCULATE THE ESTIMATED POSITION ############### 
        self.miu_hat = np.array([
            self.x + self.dt * self.v * np.cos(self.theta), #X    3x3
            self.y + self.dt * self.v * np.sin(self.theta), #Y
            self.theta + self.dt * self.w])        
        
        ############# CALCULTATE THE LINEARIZED MODEL ################
        
        self.H = np.array([[1, 0, -self.dt * self.v * np.sin(self.theta)],  #3x3
                           [0, 1, self.dt * self.v * np.cos(self.theta)],
                           [0, 0, 1]])


        ################## CALCULATE COVARIANCE #####################
        self.sigma_hat = self.H.dot(self.sigma).dot(self.H.T) + self.Q 


    def correction(self):
        dx = self.x_aruco - self.x
        dy = self.y_aruco - self.x
        p = dx **2 + dy **2 #Distance from robot to the aruco

        self.z_hat = np.array([np.sqrt(p), np.arctan2(dy, dx) - self.theta_pred]) #Observation model of the pose of the robot in respect to the aruco

        self.G = np.array([[-dx/np.sqrt(p), -dy/np.sqrt(p), 0], #Linearize observation model
                      [dy/p, -dx/p, -1]])
        
        self.Z = self.G.dot(self.sigma_hat).dot(self.G.T) + self.R  #Compute the measurement uncertainty propagation,
                                                                    #if R is big means our estimation has a lot of noise

        self.K = self.sigma_hat.dot(self.G.T).dot(inv(self.Z)) #Kalman gain determines if we choose the estimated z or the z measured

        self.miu = self.miu_hat + self.K.dot((self.coords_aruco - self.z_hat)) #K determines if we trust the estimated position or the measured pose
        
        self.sigma = (np.eye(3) - (self.K.dot(self.G))) * self.sigma_hat #Calculate covariance


    def propagate(self):
        self.miu = self.miu_hat
        self.x = self.miu[0].item()
        self.y = self.miu[1].item()
        self.theta = self.miu[2].item()
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))
        self.theta_pred = self.miu_hat[2].item()
        self.theta_pred = np.arctan2(np.sin(self.theta_pred), np.cos(self.theta_pred))
        self.sigma = self.sigma_hat


    ####################### ODOMETRY AND TRANSFORM ###############################
    def get_odom_stamped(self): 

        odom_stamped = Odometry() 
        odom_stamped.header.frame_id = "odom" 
        odom_stamped.child_frame_id = "base_link"
        odom_stamped.header.stamp = rospy.Time.now() 
        odom_stamped.pose.pose.position.x = self.x
        odom_stamped.pose.pose.position.y = self.y
        odom_stamped.pose.pose.position.z = 0.0

        quat = quaternion_from_euler(0, 0, self.theta) 
        odom_stamped.pose.pose.orientation.x = quat[0]
        odom_stamped.pose.pose.orientation.y = quat[1]
        odom_stamped.pose.pose.orientation.z = quat[2]
        odom_stamped.pose.pose.orientation.w = quat[3]

        odom_stamped.twist.twist.linear.x = self.v
        odom_stamped.twist.twist.angular.z = self.w

        # Init a 36 elements array
        odom_stamped.pose.covariance = [0.0] * 36


        # Fill the 3D covariance matrix
        odom_stamped.pose.covariance[0] = self.sigma[0][0]
        odom_stamped.pose.covariance[1] = self.sigma[0][1]
        odom_stamped.pose.covariance[5] = self.sigma[0][2]
        odom_stamped.pose.covariance[6] = self.sigma[1][0]
        odom_stamped.pose.covariance[7] = self.sigma[1][1]
        odom_stamped.pose.covariance[11] = self.sigma[1][2]
        odom_stamped.pose.covariance[30] = self.sigma[2][0]
        odom_stamped.pose.covariance[31] = self.sigma[2][1]
        odom_stamped.pose.covariance[35] = self.sigma[2][2]

        
        return odom_stamped 
    
  
    def send_transform(self, odom):
        self.tf_send = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
                    
        t.header.frame_id = "odom" 
        t.child_frame_id = "base_link"
        t.header.stamp = rospy.Time.now() 
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        quat = quaternion_from_euler(0,0,self.theta)

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_send.sendTransform(t)  #Send transform

    ###################### CALLBACKS ###########################

    def wl_cb(self, msg): 
        self.wl = msg.data


    def wr_cb(self, msg): 
        self.wr = msg.data   
    
    def goal_cb(self, msg):
        self.flag = msg.data
    
    def aruco_cb(self, msg):
        self.id_aruco = msg.data[0]
        self.d_aruco = msg.data[1]
        self.theta_aruco = msg.data[2]
        self.coords_aruco = [self.d_aruco, self.theta_aruco]
        self.id_arucos(self.id_aruco)
        self.get_aruco = True


    def id_arucos(self, id):
        x_y = {702: [0, 0.80], 701: [0, 1.60], 703: [1.73, 0.80],
               704: [2.63, 0.39], 705: [2.85, 0], 706: [2.865,2.0],
               707: [1.77, 1.22]}
        
        self.x_aruco = x_y[id][0]
        self.y_aruco = x_y[id][1]
        
if __name__ == "__main__": 
    Localisation()
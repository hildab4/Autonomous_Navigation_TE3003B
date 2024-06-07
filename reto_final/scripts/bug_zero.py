#!/usr/bin/env python3

import rospy  
from geometry_msgs.msg import Twist, PoseStamped 
from std_msgs.msg import Float32 
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion 
from sensor_msgs.msg import LaserScan   
import numpy as np 
import conditions as cnd
import gtg as gtg
import wf as wf

#This class will make the puzzlebot move to a given goal 
class AutonomousNav():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup)

        self.x_init = 0.0
        self.y_init = 0.0
        self.flag_init = True

        ############ ROBOT CONSTANTS ################  

        self.r=0.05 #wheel radius [m] 
        self.L = 0.19 #wheel separation [m] 

        ############ Variables ############### 

        self.x = 0.0 #x position of the robot [m] 
        self.y = 0.0 #y position of the robot [m] 
        self.theta = 0.0 #angle of the robot [rad] 

        ############ Variables ############### 

        self.path = np.array([[0.5 , 1.55] , [1.38 , 1.03] , [2.05 , 0.35] , [2.97 , 0.40] , [0.5 , 1.55]])
        self.index = 0
        self.x_target = self.path[self.index][0]
        self.y_target = self.path[self.index][1]
        self.goal_received = 1  #flag to indicate if the goal has been received 
        self.lidar_received = 0 #flag to indicate if the laser scan has been received 
        self.target_position_tolerance=0.05 #acceptable distance to the goal to declare the robot has arrived to it [m] 
        wf_distance = 0.2
        self.integral = 0.0
        self.prev_error = 0.0
        stop_distance = 0.08 # distance from closest obstacle to stop the robot [m] 
        v_msg=Twist() #Robot's desired speed  
        self.wr=0 #right wheel speed [rad/s] 
        self.wl=0 #left wheel speed [rad/s] 
        self.current_state = 'GoToGoal' #Robot's current state 

        ###******* INIT PUBLISHERS *******###  
        self.pub_cmd_vel = rospy.Publisher('puzzlebot_1/base_controller/cmd_vel', Twist, queue_size=1)  

        ############################### SUBSCRIBERS #####################################  

        rospy.Subscriber("puzzlebot_1/wl", Float32, self.wl_cb)  
        rospy.Subscriber("puzzlebot_1/wr", Float32, self.wr_cb)  
        rospy.Subscriber("puzzlebot_1/scan", LaserScan, self.laser_cb)
        rospy.Subscriber("puzzlebot_goal", PoseStamped, self.goal_cb) 
        rospy.Subscriber("/odom", Odometry, self.odom_cb)

        #********** INIT NODE **********###  
        freq = 30
        rate = rospy.Rate(freq) #freq Hz  
        dt = 1.0/float(freq) #Dt is the time between one calculation and the next one 
        fwf = True
        d_t1 = 0.0
        d_t = 0.0
        theta_ao = 0.0
        theta_gtg = 0.0
        self.x_tmp = np.inf
        self.y_tmp = np.inf

        ################ MAIN LOOP ################  
        while not rospy.is_shutdown():
            
            print("\nX_robot: ", self.x)
            print("Y_robot: ", self.y)
            print("X_target: ", self.x_target)
            print("Y_target: ", self.y_target, "\n")

            if self.lidar_received:
                closest_range, closest_angle = self.get_closest_object(self.lidar_msg) #get the closest object range and angle 

                if self.current_state == 'Stop': 
                    if self.goal_received and closest_range > wf_distance: 
                        print("Change to Go to goal from stop") 
                        self.current_state = "GoToGoal" 
                        self.goal_received = 0 

                    elif self.goal_received and closest_range < wf_distance: 
                        print("Change to wall following from Stop") 
                        self.current_state= "WallFollower" 

                    else: 
                        v_msg.linear.x = 0.0 
                        v_msg.angular.z =0.0 
                elif self.current_state == 'GoToGoal':  
                    if self.at_goal() or closest_range < stop_distance:  
                        print("Change to Stop from Go to goal") 
                        print(self.at_goal())
                        
                        print(np.sqrt((self.x_target-self.x)**2+(self.y_target-self.y)**2))

                        print(self.x_target)
                        print(self.y_target)
                        self.current_state = "Stop"  
                        if self.at_goal and self.index < len(self.path):
                            self.index +=1
                            self.x_target = self.path[self.index][0]
                            self.y_target = self.path[self.index][1]
                            self.current_state = "GoToGoal"  

                    elif closest_range < wf_distance: 
                        print("Change to wall following from Go to goal") 
                        self.current_state= "WallFollower" 
                        self.x_tmp = self.x
                        self.y_tmp = self.y

                    else:        
                        v_gtg, w_gtg = self.compute_gtg_control(self.x_target, self.y_target, self.x, self.y, self.theta)   
                        v_msg.linear.x = v_gtg 
                        v_msg.angular.z = w_gtg      

                elif self.current_state == 'WallFollower':
                    first_wf = True
                    theta_gtg, theta_ao = cnd.compute_angles(self.x_target, self.y_target, self.x, self.y, self.theta, closest_angle)
                    d_t = np.sqrt((self.x_target-self.x)**2+(self.y_target-self.y)**2)
                    print("Quit?: ",cnd.quit_wf_bug_two(theta_gtg, theta_ao, self.x_target, self.y_target, self.x, self.y, self.x_init, self.y_init))

                    if self.at_goal() or closest_range < stop_distance: 
                        print("Change to Stop") 
                        self.current_state = "Stop" 
                        fwf = True
                        if self.at_goal and self.index < len(self.path):
                            self.index +=1
                            self.x_target = self.path[self.index][0]
                            self.y_target = self.path[self.index][1]
                            self.current_state = "GoToGoal"  

                    #elif cnd.quit_wf_bug_two(theta_gtg, theta_ao, self.x_target, self.y_target, self.x, self.y, self.x_init, self.y_init):
                     #   print("Change to Go to goal from wall follower") 
                      #  self.current_state = "GoToGoal" 
                       # fwf = True

                    elif cnd.quit_wf_bug_two_t(self.x_target, self.y_target, self.x, self.y , self.x_tmp , self.y_tmp):
                        print("Change to Go to goal from wall follower") 
                        self.current_state = "GoToGoal" 
                        fwf = True

                    else:
                        d_t1 = np.sqrt((self.x_target-self.x)**2+(self.y_target-self.y)**2)  if fwf else d_t1
                        clk_cnt = cnd.clockwise_counter(self.x_target, self.y_target, self.x, self.y, self.theta, closest_angle) if fwf else clk_cnt
                        fwf = False
                        v_wf, w_wf = wf.compute_wf_controller(closest_angle,closest_range, 0.18, clk_cnt, dt) 
                        v_msg.linear.x = v_wf
                        v_msg.angular.z = w_wf 


            self.pub_cmd_vel.publish(v_msg)  

            rate.sleep()  

    def at_goal(self): 
        return np.sqrt((self.x_target-self.x)**2+(self.y_target-self.y)**2)<self.target_position_tolerance 


    def get_closest_object(self, lidar_msg): 
        min_idx = np.argmin(lidar_msg.ranges) 
        closest_range = lidar_msg.ranges[min_idx] 
        closest_angle = lidar_msg.angle_min + min_idx * lidar_msg.angle_increment 
        closest_angle = np.arctan2(np.sin(closest_angle), np.cos(closest_angle)) 

        return closest_range, closest_angle

    def compute_gtg_control(sefl, x_target, y_target, x_robot, y_robot, theta_robot): 
        kvmax = 0.18 #linear speed maximum gain  
        kwmax = 0.7 #angular angular speed maximum gain 

        av = 3.0 #Constant to adjust the exponential's growth rate   
        aw = 2.0 #Constant to adjust the exponential's growth rate 

        ed = np.sqrt((x_target-x_robot)**2+(y_target-y_robot)**2) 

        #Compute angle to the target position 

        theta_target = np.arctan2(y_target-y_robot,x_target-x_robot) 
        e_theta = theta_target-theta_robot 

        #limit e_theta from -pi to pi 
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta)) 
        #Compute the robot's angular speed 
        kw = kwmax*(1-np.exp(-aw*e_theta**2))/abs(e_theta) #Constant to change the speed  
        w = kw*e_theta 

        if abs(e_theta) > np.pi/8: 
            v = 0 #linear speed  

        else: 
            kv = kvmax*(1-np.exp(-av*ed**2))/abs(ed) #Constant to change the speed  
            v = kv*ed #linear speed  

        return v, w

    def laser_cb(self, msg):   
        ## This function receives a message of type LaserScan   
        self.lidar_msg = msg  
        self.lidar_received = 1  

    def wl_cb(self, wl):  
        ## This function receives a the left wheel speed [rad/s] 
        self.wl = wl.data 

    def wr_cb(self, wr):  
        ## This function receives a the right wheel speed.  
        self.wr = wr.data  

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.theta) = euler_from_quaternion(orientation_list)

        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        self.x_init = self.x if self.flag_init else self.x_init
        self.y_init = self.y if self.flag_init else self.y_init
        self.flag_init = False

    
    def goal_cb(self, goal):  
#       self.x_target = goal.pose.position.x 
#       self.y_target = goal.pose.position.y 

        self.goal_received=1 

    def cleanup(self):  
        vel_msg = Twist() 
        self.pub_cmd_vel.publish(vel_msg) 

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  
    rospy.init_node("go_to_goal_with_obstacles1", anonymous=True)  
    AutonomousNav()



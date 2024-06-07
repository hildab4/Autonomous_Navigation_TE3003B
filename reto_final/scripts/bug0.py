#!/usr/bin/env python3
import rospy 
from geometry_msgs.msg import Twist, PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion 
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Bool, Float32
import numpy as np
import tf2_ros

class Bug0():
    def __init__(self):
        rospy.init_node('bug0') 
        rospy.on_shutdown(self.cleanup)

        rospy.Subscriber("puzzlebot_1/scan", LaserScan, self.laser_cb)
        rospy.Subscriber("odom", Odometry, self.odom_cb)
        rospy.Subscriber("goalmarker", Float32MultiArray, self.goal_cb)
        rospy.Subscriber("puzzlebot_1/wl", Float32, self.wl_cb)
        rospy.Subscriber("puzzlebot_1/wr", Float32, self.wr_cb)

        self.pub_cmd_vel = rospy.Publisher("puzzlebot_1/base_controller/cmd_vel", Twist, queue_size = 1)
        self.pub_goal_reached = rospy.Publisher("goal_reached", Bool, queue_size = 1)

        dt = 0.05
        rate = rospy.Rate(1/dt)

        #############  INITIAL CONDITIONS #################
        self.xg = 0.0
        self.yg = 0.0

        self.e_theta = 0.0
        self.theta_ao = 0.0
        self.theta_fw = 0.0
        
        ############# INITIAL ROBOT POSE ###################
        self.xr = 0.0
        self.yr = 0.0
        self.tr = 0.0

        self.goal_r = False  #Goal coords recieve flag
        self.lidar_r = False #Lidar info recieve flag 

        vel_msg = Twist()

        self.wr = 0.0
        self.wl = 0.0

        self.closest_angle = 0.0
        self.closest_range = np.inf

        self.hp = 0.0    #Hitpoint
        self.tolerance = 0.02
        self.min_progress = 0.4 # CAMBIAR PARA COMPLETAR BUG0

        self.v = 0.0   #Linear vel
        self.w = 0.0   #Angular vel

        self.goal_flag = False  #Goal if flag is reached

        self.fw = 0.17   #Following wall tolerance

        self.current_state = 'GTG'
        self.previous_distance_to_goal = np.inf

        while not rospy.is_shutdown():
            ############### STATE MACHINE ###################### 
            if self.lidar_r and self.goal_r:

                self.get_closest_range()
                if self.at_goal():
                    print("Done")
                    self.goal_flag = True
                    if self.goal_flag:
                        vel_msg = Twist()
                        self.pub_cmd_vel.publish(vel_msg)
                        self.pub_goal_reached.publish(self.goal_flag)
                        rospy.sleep(0.1)

                elif self.current_state == "GTG":
                    print(self.current_state)
                    if self.closest_range <= self.fw:
                        self.hit = self.made_progress()

                        if self.closest_angle - self.e_theta <= -np.pi/2 and self.closest_angle - self.e_theta >= -np.pi:
                                self.current_state = "CW"
                                print("Changing to CW move")
                                print(self.current_state)
                                
                        elif self.closest_angle - self.e_theta >= np.pi/2 and self.closest_angle - self.e_theta <= np.pi:
                                self.current_state = "CCW"
                                print(self.closest_angle)
                                print("Changing to CCW move")
                                print(self.current_state)
                    else:
                        self.gtg_control()

                elif self.current_state == "CW": #Turn Right
                    print(self.current_state)
                    self.fw_control(True)

                    print("Separado de pared? " + str(self.made_progress() < self.closest_range + self.fw))

                    print("Angulo despejado? " + str((self.theta_ao - self.e_theta) < -np.pi/3))


                    if self.made_progress() < self.closest_range + self.fw and self.theta_ao - self.e_theta < -np.pi/3:  #90° 
                        self.current_state = "GTG"
                    elif self.at_goal():
                        self.current_state = "Stop"
                    else:
                        self.fw_control(True)

                elif self.current_state == "CCW": #Turn Left
                    print(self.current_state)
                    self.fw_control(False)

                    print("Angulo a pared mas cercana " + str(self.closest_angle))
                    print("Angulo del goal al robot " + str(self.e_theta))

                    print("Separado de pared? " + str(self.made_progress() < self.closest_range + self.fw))

                    print("Angulo despejado? " + str((self.theta_ao - self.e_theta) < np.pi/3))

                    if self.made_progress() < self.closest_range + self.fw and self.theta_ao - self.e_theta < np.pi/3: 
                        self.current_state = "GTG"
                    elif self.at_goal():
                        self.current_state = "Stop"
                    else:
                        self.fw_control(False)

            print("X robot: " + str(self.xr) + " X goal: " + str(self.xg))
            print("Y robot: " + str(self.yr) + " Y goal: " + str(self.yg))
            vel_msg.linear.x = self.v
            vel_msg.angular.z = self.w
            self.pub_cmd_vel.publish(vel_msg)
            rate.sleep()


    def at_goal(self):
        return (abs(self.xr - self.xg) < self.tolerance) and (abs(self.yr - self.yg) < self.tolerance)
    
    def goal_cb(self, msg):
        self.xg = msg.data[0]
        self.yg = msg.data[1]
        self.goal_r = True
    
    def made_progress(self):
        return np.sqrt((self.xg - self.xr) ** 2 + (self.yg - self.yr) ** 2)

    def get_closest_range(self):
        new_angle_min = self.lidar_msg.angle_min

        min_idx = np.argmin(self.lidar_msg.ranges)
        self.closest_range = self.lidar_msg.ranges[min_idx]
        closest_angle = new_angle_min + min_idx * self.lidar_msg.angle_increment
        closest_angle += np.pi
        # limit the angle to [-pi, pi]
        self.closest_angle = np.arctan2(np.sin(closest_angle), np.cos(closest_angle))

    def gtg_control(self):
        kv_m = 0.1
        kw_m = 1.6

        av = 2.0
        aw = 2.0

        e_d = np.sqrt((self.xg - self.xr) ** 2 + (self.yg - self.yr) ** 2)
        tg = np.arctan2(self.yg - self.yr, self.xg - self.xr)
        self.e_theta = tg - self.tr
        self.e_theta = np.arctan2(np.sin(self.e_theta), np.cos(self.e_theta))

        kw = kw_m * (1 - np.exp(-aw * self.e_theta ** 2)) / abs(self.e_theta)        
        self.w = kw * self.e_theta
        print("W: ", self.w)

        if abs(self.e_theta) > np.pi/4:
             self.v = 0.0
        else:
            kv = kv_m * (1 - np.exp(-av * e_d ** 2))/e_d
            self.v = kv * e_d + 0.02
        print("V: ", self.v)

    def fw_control(self, clockwise):
        self.closest_angle = np.arctan2(np.sin(self.closest_angle), np.cos(self.closest_angle))
        theta_ao = self.closest_angle
        self.theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))
        if clockwise:
            theta_fw = (np.pi / 2) + self.theta_ao
        else:
            theta_fw = self.tr / 2 - self.theta_ao
            
        self.theta_fw = np.arctan2(np.sin(theta_fw), np.cos(theta_fw))

        print("Theta fw: " + str(self.theta_fw))

        kw = 1.2
        self.v = 0.04
        self.w = kw * self.theta_fw

    def laser_cb(self, scan):
        self.lidar_msg = scan
        self.lidar_r = True

    def odom_cb(self, msg):
        orientation = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
            ]
        euler = euler_from_quaternion(orientation)

        self.xr = msg.pose.pose.position.x
        self.yr = msg.pose.pose.position.y
        self.tr = euler[2]

    def wr_cb(self, msg):
        self.wr = msg

    def wl_cb(self, msg):
        self.wl = msg

    def cleanup(self): 
            #This function is called just before finishing the node 
            # You can use it to clean things up before leaving 
            # Example: stop the robot before finishing a node.   
            vel_msg = Twist()
            self.pub_cmd_vel.publish(vel_msg)

############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    Bug0()  

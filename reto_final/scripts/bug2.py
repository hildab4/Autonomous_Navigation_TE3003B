#!/usr/bin/env python3
import rospy 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion 
from sensor_msgs.msg import LaserScan
import numpy as np

class Bug2():
    def __init__(self):
        rospy.init_node('bug2') 
        rospy.on_shutdown(self.cleanup)

        rospy.Subscriber("puzzlebot_1/scan", LaserScan, self.laser_cb)
        rospy.Subscriber("puzzlebot_1/base_controller/odom", Odometry, self.odom_cb)

        self.pub_cmd_vel = rospy.Publisher("puzzlebot_1/base_controller/cmd_vel", Twist, queue_size = 1)

        rate = rospy.Rate(20)
        dt = 1.0 / 10

        self.xg = 0.0
        self.yg = 0.0

        self.e_theta = 0.0
        self.theta_ao = 0.0
        self.theta_fw = 0.0
        
        self.xr = 0.0
        self.yr = 0.0
        self.tr = 0.0

        self.goal_r = False
        self.lidar_r = False

        vel_msg = Twist()

        self.closest_angle = 0.0
        self.closest_range = np.inf

        self.hit_point = None
        self.leave_point = None
        self.tolerance = 0.1
        self.min_progress = 0.3 # Change this for Bug2

        self.v = 0.0
        self.w = 0.0

        self.fw = 0.33

        self.current_state = 'GTG'
        self.set_point_cb()
        self.previous_distance_to_goal = np.inf

        while not rospy.is_shutdown():
             if self.lidar_r and self.goal_r:
                  self.get_closest_range()
                  print(self.closest_range)
                  if self.at_goal():
                       print("Done")
                       self.v = 0.0
                       self.w = 0.0
                  elif self.current_state == "GTG":
                       print(self.current_state)
                       if self.closest_range <= self.fw:
                            self.hit_point = (self.xr, self.yr)
                            if abs(self.closest_angle - self.e_theta) <= np.pi/4:
                                 self.current_state = "CW"
                                 print(self.current_state)
                            elif abs(self.closest_angle - self.e_theta) > np.pi/4:
                                 self.current_state = "CCW"
                                 print(self.current_state)
                       else:
                            self.gtg_control()
                  elif self.current_state == "CW":
                       self.fw_control(True)
                       if self.on_m_line():
                            self.current_state = "GTG"
                       elif self.at_goal():
                            self.current_state = "Stop"
                       else:
                            self.fw_control(True)
                  elif self.current_state == "CCW":
                       self.fw_control(False)
                       if self.on_m_line():
                            self.current_state = "GTG"
                       elif self.at_goal():
                            self.current_state = "Stop"
                       else:
                            self.fw_control(False)

             vel_msg.linear.x = self.v
             vel_msg.angular.z = self.w
             self.pub_cmd_vel.publish(vel_msg)
             rate.sleep()

    def at_goal(self):
        return (abs(self.xr - self.xg) < self.tolerance) and (abs(self.yr - self.yg) < self.tolerance)
    
    def made_progress(self):
        return np.sqrt((self.xg - self.xr) ** 2 + (self.yg - self.yr) ** 2) < self.previous_distance_to_goal

    def on_m_line(self):
        a = self.yr
        b = 0 - self.xr
        c = self.xr * 0 - 0 * self.yr
        error = np.abs(a * self.xg + b * self.yg + c) / np.sqrt(a * a + b * b)
        
        if error <= 0.08:
             return True
        return False

    def get_closest_range(self):
        new_angle_min = self.lidar_msg.angle_min
        min_idx = np.argmin(self.lidar_msg.ranges)
        self.closest_range = self.lidar_msg.ranges[min_idx]
        closest_angle = new_angle_min + min_idx * self.lidar_msg.angle_increment
        self.closest_angle = np.arctan2(np.sin(closest_angle), np.cos(closest_angle))

    def gtg_control(self):
        kv_m = 0.25
        kw_m = 1.0

        av = 2.0
        aw = 2.0

        e_d = np.sqrt((self.xg - self.xr) ** 2 + (self.yg - self.yr) ** 2)
        tg = np.arctan2(self.yg - self.yr, self.xg - self.xr)
        e_theta = tg - self.tr
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))

        kw = kw_m * (1 - np.exp(-aw * e_theta ** 2)) / abs(e_theta)        
        self.w = kw * e_theta

        if abs(e_theta) > np.pi/8:
             self.v = 0.0
        else:
            kv = kv_m * (1 - np.exp(-av * e_d ** 2))/abs(e_d)
            self.v = kv * e_d

    def fw_control(self, clockwise):
        self.closest_angle = np.arctan2(np.sin(self.closest_angle), np.cos(self.closest_angle))
        theta_ao = self.closest_angle - np.pi
        self.theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))
        if clockwise:
            theta_fw = -np.pi / 2 + self.theta_ao
        else:
            theta_fw = np.pi / 2 + self.theta_ao
        self.theta_fw = np.arctan2(np.sin(theta_fw), np.cos(theta_fw))

        kw = 3.0
        self.v = 0.15
        self.w = kw * self.theta_fw

    def laser_cb(self, msg):
        self.lidar_msg = msg
        self.lidar_r = True

    def set_point_cb(self):
        # Set the goal coordinates
        self.xg = 2.3
        self.yg = 2.5
        self.goal_r = True

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

    def cleanup(self): 
            vel_msg = Twist()
            self.pub_cmd_vel.publish(vel_msg)

if __name__ == "__main__":  
    Bug2()

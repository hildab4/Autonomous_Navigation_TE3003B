#!/usr/bin/env python3  

from fiducial_msgs.msg import FiducialTransformArray
import tf
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray
import rospy
import numpy as np


class ArucoDetect():
    def __init__(self):
        rospy.init_node("DetectR")

        rospy.Subscriber("fiducial_transforms", FiducialTransformArray, self.fiducial_cb)

        self.aruco_pub = rospy.Publisher("aruco_topic", Float32MultiArray, queue_size=1)

        self.aruco = FiducialTransformArray()
        #self.needed_id = 721

        
        self.aruco_info = [0.0, 0.0, 0.0] #ID DISTANCIA THETA
        self.quat = [0.0, 0.0, 0.0, 0.0]
        self.T = [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]
        self.rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if len(self.aruco.transforms) >= 1:
                #found_id = self.aruco.transforms[0].fiducial_id
    
                #if found_id == self.needed_id:
                '''
                self.pos = [round(self.aruco.transforms[0].transform.translation.x, 5),
                            round(self.aruco.transforms[0].transform.translation.y, 5),
                            round(self.aruco.transforms[0].transform.translation.z, 5),
                            1]'''

                self.quat = [round(self.aruco.transforms[0].transform.rotation.x, 5),
                        round(self.aruco.transforms[0].transform.rotation.y, 5),
                        round(self.aruco.transforms[0].transform.rotation.z, 5),
                        round(self.aruco.transforms[0].transform.rotation.w, 5)]

                euler = euler_from_quaternion(self.quat)
                yaw = round(euler[2], 5)

                self.pos = [round(self.aruco.transforms[0].transform.translation.x, 5),
                            round(self.aruco.transforms[0].transform.translation.y, 5),
                            round(self.aruco.transforms[0].transform.translation.z, 5),
                            1]

                self.transform_frame()
                
                dist = np.sqrt(self.robotParuco[0] ** 2 + self.robotParuco[1] ** 2)
                theta = np.arctan2(self.robotParuco[1], self.robotParuco[0])

                print("\n=========== Position of Aruco with respect to robot frame ===========")
                print("\nPosition in x: " + str(self.robotParuco[0]))
                print("\nPosition in y: " + str(self.robotParuco[1]))
                print("\nPosition in z: " + str(self.robotParuco[2]))
                print("\nOrientation in yaw rad: " + str(theta))
                print("\nDistancia al aruco: " + str(dist))

                self.aruco_info[0] = self.aruco.transforms[0].fiducial_id
                self.aruco_info[1] = dist
                self.aruco_info[2] = theta

                self.aruco_pub.publish(self.aruco_info)

                #print("\nNeeded ID not found")
            self.rate.sleep()   

    def fiducial_cb(self, msg):
        self.aruco = msg

    def transform_frame(self):  
        self.T = [[0, 0, 1, 0.12],
                  [1, 0, 0, 0],
                  [0, 1, 0, 0.09],
                  [0, 0, 0, 1]]
        self.robotParuco = np.dot(self.T, self.pos)

if __name__ == "__main__":
    rospy.init_node("DetectR")
    ArucoDetect()
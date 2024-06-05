#!/usr/bin/env python3

import rospy
import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PointStamped

def broadcaster_and_transformer():
    rospy.init_node('tf_broadcaster_and_transformer')

    # Set up the transform broadcaster
    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)  # 10 Hz

    while not rospy.is_shutdown():
        # Broadcasting the transform
        position = (1.0, 2.0, 0.0)  # x, y, z
        orientation = quaternion_from_euler(0.0, 0.0, 1.57)  # roll, pitch, yaw

        br.sendTransform(position,
                         orientation,
                         rospy.Time.now(),
                         "base_link",
                         "world")

        # Transforming a point
        point_in_robot_frame = PointStamped()
        point_in_robot_frame.header.frame_id = "base_link"
        point_in_robot_frame.header.stamp = rospy.Time.now()
        point_in_robot_frame.point.x = 1.0
        point_in_robot_frame.point.y = 0.0
        point_in_robot_frame.point.z = 0.0

        try:
            point_in_world_frame = listener.transformPoint('world', point_in_robot_frame)
            rospy.loginfo("Point in world frame: %s", point_in_world_frame)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Transform failed: %s", e)

        rate.sleep()

if __name__ == '__main__':
    try:
        broadcaster_and_transformer()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
import rospy
import numpy as np
import math
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_from_matrix, quaternion_multiply
from nav_msgs.msg import Odometry

# Topic given by rtabmap in odom frame
rtabmap_original_odom_topic = "/rtabmap/ugv_odom"
# Odom in base link topic
rtabmap_odom_topic = "/rtabmap/odom"

target_frame_id = "/ugv_base_link"
source_frame_id = "/ugv_odom" # /camera_link

# Create the odometry message
odom = Odometry()
odom_original = Odometry()

def odomCallback(odom_msg):
    global odom_original
    odom_original = odom_msg
    # odom.header.stamp = odom_msg.header.stamp


if __name__ == '__main__':
    rospy.init_node('visual_odom_frame_transformer')

    listener = tf.TransformListener()

    # Create the odom publisher
    rtabmap_odom_pub = rospy.Publisher(rtabmap_odom_topic, Odometry, queue_size=1)

    # Create the odometry subscriber
    rospy.Subscriber(rtabmap_original_odom_topic, Odometry, odomCallback)

    listener.waitForTransform(target_frame_id, source_frame_id, rospy.Time(), rospy.Duration(100.0))

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform(target_frame_id, source_frame_id, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # Apply the transformation from odom frame to base link frame
        odom_x =  odom_original.pose.pose.position.x
        odom_y =  odom_original.pose.pose.position.y
        odom_z =  odom_original.pose.pose.position.z
        odom_rx = odom_original.pose.pose.orientation.x
        odom_ry = odom_original.pose.pose.orientation.y
        odom_rz = odom_original.pose.pose.orientation.z
        odom_rw = odom_original.pose.pose.orientation.w

        # Tranform the rot and trans to a matrix
        R_L = quaternion_matrix([rot[0],rot[1],rot[2],rot[3]])
        R_L[0][3] =  trans[0]
        R_L[1][3] =  trans[1]
        R_L[2][3] =  trans[2]
        R_L = np.linalg.inv(R_L)

        # Matrix with the original message rotation and translation
        R_I = quaternion_matrix([odom_rx, odom_ry, odom_rz, odom_rw])
        R_I[0][3] =  odom_x
        R_I[1][3] =  odom_y
        R_I[2][3] =  odom_z

        # Matrix with the final rotation and translation
        R_F = R_I.dot(R_L)
        # Final quaternion matrix
        quat  = quaternion_from_matrix(R_F)

        # Fill the odometry message
        odom.header.stamp = odom_original.header.stamp
        odom.header.frame_id = source_frame_id
        odom.child_frame_id = target_frame_id
        odom.pose.pose = Pose(Point(R_F[0][3],R_F[1][3],R_F[2][3]),Quaternion(quat[0],quat[1],quat[2],quat[3]))

        # Publish the odometry message
        rtabmap_odom_pub.publish(odom)

        rate.sleep()

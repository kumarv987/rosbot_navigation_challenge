#!/usr/bin/env python3

import rospy
import math
import tf
import tf2_ros
import tf_conversions
import geometry_msgs
from geometry_msgs.msg import PoseStamped

def transform():
    listener = tf.TransformListener()
    pub = rospy.Publisher('/path',PoseStamped,queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #rospy.loginfo("Time:" + str(rospy.Time.now()))

        delay = 0.2

        # EXAMPLE
        # Lookup a tranformation
        try:
            dest = 'map'
            src = 'base_link'
            (trans, rot) = listener.lookupTransform(dest, src, rospy.Time.now() - rospy.Duration(delay))
            #rospy.loginfo("Transform received: " + src + " -> " + dest + ": Translation: (" + str(trans) + "), Rotation: (" + str(rot) + ")" )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("could not load transform: " + src + " -> " + dest)

        # EXAMPLE
        # Transform a POSE
        try:
            # Create the POSE to transform (1m in x)
            pose = geometry_msgs.msg.Pose()
            pose.position.x = 0

            # Create Stamped pose
            # Need to provide time, and source frame.
            # This is the pose that will be transformed
            pstamped = geometry_msgs.msg.PoseStamped()
            pstamped.header.stamp = rospy.Time.now() - rospy.Duration(delay)
            pstamped.header.frame_id = src
            pstamped.pose = pose

            # Tranform a given pose
            transposed = listener.transformPose(dest, pstamped)
            
            #Publish the robot's transformed pose to /Path topic
            rate = rospy.Rate(5)
            message = PoseStamped()
            message = transposed
            pub.publish(message)
            rate.sleep()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("could not load transform: " + src + " -> " + dest)


        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('transform', anonymous=True)
        transform()
    except rospy.ROSInterruptException:
        pass

    exit(0)


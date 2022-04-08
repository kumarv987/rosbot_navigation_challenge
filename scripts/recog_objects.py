#!/usr/bin/env python3

import math
import rospy
from std_msgs.msg import Float32MultiArray, Header, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker

list_marker_id = []
enteredStart = True

def callback(img_msg,pubHaz,pubStart):
    if (len(img_msg.data) > 0):    
        img_id = int(img_msg.data[0]) - 1

        print("CHECK ID: ", img_id)
        
        global list_marker_id
        if (img_id > 0 and img_id not in list_marker_id):
            
            #if (img_id not in list_marker_id):
            print("Hazard Detected")
            list_marker_id.append(img_id)

            # Get the distance from laser at anlge 0 of the hazard marker
            laser_subs = rospy.wait_for_message('/scan', LaserScan)
            img_dist = laser_subs.ranges[0]
            
            # Get robots position with map_frame reference
            robot_pos_subs = rospy.wait_for_message('/path', PoseStamped)
            pos = robot_pos_subs.pose.position
            orientation = robot_pos_subs.pose.orientation
            curr_x = pos.x
            curr_y = pos.y

            orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
            yaw = yaw * 180 / math.pi
            yaw = yaw % 360 

            # # cartesian angle
            curr_angle = yaw + 90
            curr_angle = -curr_angle % 360

            # # //convert to radians
            angle_radians = yaw * (math.pi / 180)

            # # //get x, y from dist, dir, and current pos
            image_x = curr_x + (img_dist * math.cos(angle_radians))
            image_y = curr_y + (img_dist * math.sin(angle_radians))

            #Defining the marker object to send info on /hazard topic
            markerObject = Marker()
            markerObject.header.frame_id = 'map'
            markerObject.id = img_id
            markerObject.pose.position.x = image_x
            markerObject.pose.position.y = image_y
            markerObject.pose.position.z = 0.2
            markerObject.pose.orientation.x = 0.0        
            markerObject.pose.orientation.y = 0.0 
            markerObject.pose.orientation.z = 0.0
            markerObject.pose.orientation.w = 1.0 
            markerObject.scale.x = 0.3
            markerObject.scale.y = 0.1
            markerObject.scale.z = 0.1
            markerObject.color.a = 1.0
            markerObject.color.r = 0.0
            markerObject.color.g = 1.0
            markerObject.color.b = 0.1
            markerObject.type = Marker.SPHERE
            markerObject.action = Marker.ADD
                            
            pubHaz.publish(markerObject)
            print("PUBLISHED A HAZARD")

        global enteredStart
        if (img_id == 0 and enteredStart) :
            # Initiate the move
            enteredStart = False
            print("Start sign Detected")
            pubStart.publish("START")

# Short ROS Node method
if __name__ == '__main__':
    try:
        rospy.init_node('recog_objects', anonymous=True)
        pubHaz = rospy.Publisher('/hazards', Marker, queue_size=1)
        pubStart = rospy.Publisher('/startDetected',String, queue_size=1)
        while not rospy.is_shutdown():
            #rospy.Subscriber("/objects", Float32MultiArray, callback)
            obj_message = rospy.wait_for_message('/objects',Float32MultiArray)
            callback(obj_message,pubHaz,pubStart)

    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3

import math
import rospy
from std_msgs.msg import Float32MultiArray, Header, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker

list_marker_id = []
#started = False

def callback(img_msg):
    
    if len(img_msg.data) > 0:
        img_id = int(img_msg.data[0]) - 1

        print("CHECK ID: ", img_id)
        #global started

        if (img_id > 0):
            
            if (img_id not in list_marker_id):
                print("Detected a SIGN")
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

                # cartesian angle
                curr_angle = yaw + 90
                curr_angle = -curr_angle % 360

                # //convert to radians
                angle_radians = yaw * (math.pi / 180)

                # //get x, y from dist, dir, and current pos
                image_x = curr_x + (img_dist * math.cos(angle_radians))
                image_y = curr_y + (img_dist * math.sin(angle_radians))

                #Defining the marker object to send info on /hazard topic
                markerObject = Marker()
                markerObject.header.frame_id = 'map'
#                markerObject.stamp =  rospy.Time.now()
                markerObject.id = img_id
                markerObject.pose.position.x = image_x
                markerObject.pose.position.y = image_y
                markerObject.pose.position.z = 0.2
                markerObject.pose.orientation.x = 0.0        
                markerObject.pose.orientation.y = 0.0 
                markerObject.pose.orientation.z = 0.0           
                markerObject.pose.orientation.w = 1.0 
                markerObject.scale.x = 0.1
                markerObject.scale.y = 0.1
                markerObject.scale.z = 0.1
                markerObject.color.a = 1.0
                                
                print(markerObject)
                pubHaz = rospy.Publisher('/hazards', Marker, queue_size=1)
                pubHaz.publish(markerObject)
                print("PUBLISHED A HAZARD")

        if (img_id == 0):
            # Initiate the move
            #started = True
            print("Start sign Detected")
            pub = rospy.Publisher("/startDetected", String)
            pub.publish("START")


# Short ROS Node method
if __name__ == '__main__':
    try:
        rospy.init_node('recog_objects', anonymous=True)
        while True:
            rospy.Subscriber("/objects", Float32MultiArray, callback)

    except rospy.ROSInterruptException:
        pass

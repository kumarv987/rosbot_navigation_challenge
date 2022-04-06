#!/usr/bin/env python3

import rospy

import actionlib
from geometry_msgs.msg import (
    PoseStamped,
    Twist
)
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf import TransformListener
from geometry_msgs.msg import Twist

class Waypoint():
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.map_frame = rospy.get_param("~map_frame", 'map')
        self.timeout = rospy.get_param("~timeout", 30)

        # Drive publisher
        self.pub_drive = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()
        self.cmd.linear.x = 0
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0


    def execute(self):
        # Create waypoints
        waypoint_x = 5
        waypoint_y = 5

        # Ensure move_base action client server is available
        rospy.loginfo('Connecting to move_base...')
        timer = self.client.wait_for_server(rospy.Duration(self.timeout))
        if not timer:
            rospy.logerr("Could not connect to move base server, terminating")
            return
        rospy.loginfo('Connected to move_base.')

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.pose.position.x = waypoint_x
        goal.target_pose.pose.position.y = waypoint_y
        goal.target_pose.pose.orientation.w = 1
        rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' %
                      (goal.target_pose.pose.position.x, goal.target_pose.pose.position.y))
        rospy.loginfo(
            "To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")


        # Iterate through waypoints
        while not rospy.is_shutdown():
            startTime = rospy.get_rostime()
            # Send the command to action server to navigate the map
            self.client.send_goal(goal)
            while (rospy.get_rostime().secs - startTime.secs) < rospy.Duration(15).secs:
                print("Time loop X seonds for cancel_goal")
            self.client.cancel_goal()

            startTimeFinishedMoveBase = rospy.get_rostime()
            while (rospy.get_rostime().secs - startTimeFinishedMoveBase.secs) < rospy.Duration(5).secs:
                print("Finished Move Base wait 5 seconds")

            speed=0.4
            max_duration = 20
            t_start = rospy.Time.now()
            rate1 = rospy.Rate(20)
            stop = False
            while (not stop):
                self.cmd.linear.x = 0
                self.cmd.angular.z = speed
                self.pub_drive.publish(self.cmd)
                print("Rotate for 20 seconds")
                rate1.sleep()
                
                # Check time
                t_now = rospy.Time.now()
                if t_now - t_start > rospy.Duration(max_duration):
                    stop = True

            startTimeFinishedRotation = rospy.get_rostime()
            while (rospy.get_rostime().secs - startTimeFinishedRotation.secs) < rospy.Duration(5).secs:
                print("Done Rotation now wait 5 seconds")

            
        # Execute is complete
        return 'completed'


# Short ROS Node method
if __name__ == '__main__':
    try:
        rospy.init_node('rosbot_waypoint', anonymous=True)

        wp = Waypoint()
        wp.execute()
    except rospy.ROSInterruptException:
        pass

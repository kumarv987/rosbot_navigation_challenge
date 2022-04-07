import actionlib
import move_base
import rospy
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class MoveGoal():
    def __init__(self, x, y):
        # Get an action client
        self.move_base = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction)
        self.move_base.wait_for_server()

        # Define the goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.w = 1

        self.timeout = rospy.get_param("~timeout", 30)

    def execute(self):
        print("moving")
        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(self.goal)

        # Allow 1 minute to get there
        finished_within_time = self.move_base.wait_for_result(
            rospy.Duration(60))

        done = False
        while not done:
            # If we don't get there in time, abort the goal
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
                done = True
            else:
                # target achieved
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Target Achieved")
                    done = True

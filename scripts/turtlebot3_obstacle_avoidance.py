#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class Turtlebot3ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('turtlebot3_obstacle_avoidance', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        self.current_pose = None
        self.current_orientation = None

    def odom_callback(self, data):
        self.current_pose = data.pose.pose.position
        orientation = data.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_orientation = yaw

    def move_to_goal(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0

        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()

    def run(self):
        rate = rospy.Rate(10)
        goals = [(2, 2), (-2, 2), (-2, -2), (2, -2)]
        current_goal = 0

        while not rospy.is_shutdown():
            if self.current_pose is not None:
                goal_x, goal_y = goals[current_goal]
                self.move_to_goal(goal_x, goal_y)
                current_goal = (current_goal + 1) % len(goals)

            rate.sleep()

if __name__ == '__main__':
    try:
        controller = Turtlebot3ObstacleAvoidance()
        controller.run()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

import rospy
import actionlib                    # Imports the actionlib library used for implementing simple actions
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Float64MultiArray, Float64


class MoveBaseTiagoClientClass(object):
    # Class constructor
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def send_goal(self, current_goal):
        # print('Waiting for server...')
        #self.client.wait_for_server()

        # Here I can take the correct parameters according to the goal received and the state_index
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = current_goal.parameters.data[0]
        goal.target_pose.pose.position.y = current_goal.parameters.data[1]
        goal.target_pose.pose.orientation.x = current_goal.parameters.data[3]
        goal.target_pose.pose.orientation.y = current_goal.parameters.data[4]
        goal.target_pose.pose.orientation.z = current_goal.parameters.data[5]
        goal.target_pose.pose.orientation.w = current_goal.parameters.data[6]
        # rospy.loginfo('Goal sent!')
        print('Sending move base goal', goal)
        self.client.send_goal(goal)
        
        # wait = self.client.wait_for_result()
        # if not wait:
        #     rospy.logerr("Action server not available!")
        #     rospy.signal_shutdown("Action server not available!")
        # else:
        #     return self.client.get_state()

    def cancel_goal(self):
        self.client.cancel_goal()
        # print('Requested to cancel goal to moveBase')

    def get_state(self):
        return self.client.get_state()
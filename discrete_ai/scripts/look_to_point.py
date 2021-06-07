#!/usr/bin/env python

""" 
Created on 06-01-2021

@Author: Stefan Bonhof

look_to_point.py contains... a class that uses 
the point_head_action server to make tiago look to 
a point (currently any tf link) using the action
instead of publishing to the  
/head_controller/point_head_action/goal topic 
"""

# ROS imports
import rospy
import actionlib
from rospy import client
import tf.transformations

# ROS messages imports
from control_msgs.msg import PointHeadActionGoal, PointHeadAction
from geometry_msgs.msg import PointStamped

        
class LookToPoint():

    def __init__(self):
        """Node to look at certain points"""

        # Launch Client and wait for server
        self.client = actionlib.SimpleActionClient('/head_controller/point_head_action', PointHeadAction)
        rospy.loginfo('Waiting for server for head control...')
        self.client.wait_for_server()
        rospy.loginfo('Server found...')
        self.tf_l = tf.TransformListener()
        rospy.sleep(2.0)

    def run(self, point):
        r = rospy.Rate(10)
        goal = PointHeadActionGoal()
        goal.header.frame_id = "/base_link"
        goal.goal.max_velocity = 1.0
        goal.goal.min_duration = rospy.Duration(0.2)
        goal.goal.target.header.frame_id = "/base_link"
        goal.goal.pointing_axis.x = 1.0
        goal.goal.pointing_frame = "/head_2_link"

    
        if isinstance(point, str):
            ## Look at target frame:
            target_frame = point
            ps = PointStamped()
            ps.header.stamp = self.tf_l.getLatestCommonTime("/base_link", target_frame)
            ps.header.frame_id = target_frame
            transform_ok = False
            while not transform_ok and not rospy.is_shutdown():
                try:
                    target_frame_ps = self.tf_l.transformPoint("/base_link", ps)
                    transform_ok = True
                    goal.goal.target.point = target_frame_ps.point
                except tf.ExtrapolationException as e:
                    rospy.logwarn("Exception on transforming point... trying again \n(" + str(e) + ")")
                    rospy.sleep(0.01)
                    ps.header.stamp = self.tf_l.getLatestCom

        else:
            ## Look at point in space:
            goal.goal.target.point = point


        rospy.loginfo('Sending goal...')
        self.client.send_goal(goal.goal)  
        rospy.loginfo('Waiting for result...')
        self.client.wait_for_result(rospy.Duration.from_sec(5.0))
        rospy.loginfo('Goal achieved...')
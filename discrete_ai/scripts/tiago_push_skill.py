#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from visualization_msgs.msg import Marker
from tiago_moveit import tiagoMoveit
from tiago_gripper import GripperControl


class tiagoPush(object):
    def __init__(self):
        # Pushing with left arm. The current skill is just a set of waypoints but it can be substituted here with more advanced skills
        self.tiago_moveit = tiagoMoveit('left')
        self.tiago_gripper = GripperControl('left')
        
        # Poses to perform the push
        self.pose_push = geometry_msgs.msg.Pose()
        self.pose_rest = geometry_msgs.msg.Pose()
        self.pose_prepush = geometry_msgs.msg.Pose()

        self.pose_push = geometry_msgs.msg.Pose()
        self.pose_push.orientation.x = 0.707
        self.pose_push.orientation.y = 0.0
        self.pose_push.orientation.z = 0.0
        self.pose_push.orientation.w = 0.707
        self.pose_push.position.x = 0.7
        self.pose_push.position.y = 0.2
        self.pose_push.position.z = 0.95

        self.pose_rest = geometry_msgs.msg.Pose()
        self.pose_rest.orientation.x = 0.707
        self.pose_rest.orientation.y = 0.0
        self.pose_rest.orientation.z = 0.0
        self.pose_rest.orientation.w = 0.707
        self.pose_rest.position.x = 0.2
        self.pose_rest.position.y = 0.2
        self.pose_rest.position.z = 0.8

        self.pose_prepush = geometry_msgs.msg.Pose()
        self.pose_prepush.orientation.x = 0.707
        self.pose_prepush.orientation.y = 0.0
        self.pose_prepush.orientation.z = 0.0
        self.pose_prepush.orientation.w = 0.707
        self.pose_prepush.position.x = 0.4
        self.pose_prepush.position.y = 0.2
        self.pose_prepush.position.z = 0.95

    def send_goal(self, goal):

        self.tiago_gripper.run('open')
        self.tiago_moveit.run( self.pose_prepush)
        self.tiago_moveit.run( self.pose_push)
        self.tiago_moveit.run( self.pose_prepush)
        self.tiago_moveit.run( self.pose_rest)
        self.tiago_gripper.run('open')
        print('Object pushed')

    def get_state(self):
        # To modify with the actual return status if available
        return 1


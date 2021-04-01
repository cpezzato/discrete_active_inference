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


class tiagoPlace(object):
    def __init__(self):
        self.tiago_moveit = tiagoMoveit('right')
        self.tiago_gripper = GripperControl('right')

        self.pose_preplace = geometry_msgs.msg.Pose()
        self.pose_rest = geometry_msgs.msg.Pose()
        self.pose_place = geometry_msgs.msg.Pose()

        self.pose_place = geometry_msgs.msg.Pose()
        self.pose_place.orientation.x = 0.707
        self.pose_place.orientation.y = 0.0
        self.pose_place.orientation.z = 0.0
        self.pose_place.orientation.w = 0.707
        self.pose_place.position.x = 0.5
        self.pose_place.position.y = -0.2
        self.pose_place.position.z = 0.90

        self.pose_rest = geometry_msgs.msg.Pose()
        self.pose_rest.orientation.x = 0.707
        self.pose_rest.orientation.y = 0.0
        self.pose_rest.orientation.z = 0.0
        self.pose_rest.orientation.w = 0.707
        self.pose_rest.position.x = 0.2
        self.pose_rest.position.y = -0.2
        self.pose_rest.position.z = 0.8

        self.pose_preplace = geometry_msgs.msg.Pose()
        self.pose_preplace.orientation.x = 0.707
        self.pose_preplace.orientation.y = 0.0
        self.pose_preplace.orientation.z = 0.0
        self.pose_preplace.orientation.w = 0.707
        self.pose_preplace.position.x = 0.5
        self.pose_preplace.position.y = -0.2
        self.pose_preplace.position.z = 0.95


    def send_goal(self, goal):

        self.tiago_moveit.run(self.pose_preplace)
        self.tiago_moveit.run(self.pose_place)
        self.tiago_gripper.run('open')
        self.tiago_moveit.run(self.pose_preplace)
        self.tiago_moveit.run(self.pose_rest)


    def get_state(self):
        # To modify with the actual return status 
        return 1


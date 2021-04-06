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
from aruco_msgs.msg import Marker
from aruco_msgs.msg import MarkerArray
from look_to_point import LookToPoint

class tiagoPlace(object):
    def __init__(self):

        # Aruco
        self._aruco_pose = geometry_msgs.msg.PoseStamped()
        self._aruco_id = 0
        self._aruco_found = False
        
        # Move it and gripper
        self.tiago_moveit = tiagoMoveit('right')
        self.tiago_gripper = GripperControl('right')

        self.pose_preplace = geometry_msgs.msg.Pose()
        self.pose_rest = geometry_msgs.msg.Pose()
        self.pose_place = geometry_msgs.msg.Pose()
        
        self.pose_rest.position.x = 0.2
        self.pose_rest.position.y = -0.2
        self.pose_rest.position.z = 0.8
        self.pose_rest.orientation.x = 0.707
        self.pose_rest.orientation.y = 0.0
        self.pose_rest.orientation.z = 0.0
        self.pose_rest.orientation.w = 0.707

        # Fixed orientation for place and pre-place
        self.pose_place.orientation = copy.deepcopy(self.pose_rest.orientation)
        self.pose_preplace.orientation = copy.deepcopy(self.pose_rest.orientation)

        # Initial pose is the rest pose
        self._aruco_pose.pose.position = copy.deepcopy(self.pose_rest.position)
        self._aruco_pose.pose.orientation = copy.deepcopy(self.pose_rest.orientation)

        # Variables for better marker detection
        self.counter = 0    
        self.grasping = False

        # Aruco sub
        self._aruco_sub = rospy.Subscriber("aruco_marker_publisher/markers", MarkerArray, self.aruco_cb)
        


    def aruco_cb(self, msg):
        # Callback to update pose of detected aruco marker on the object
        for marker in msg.markers:
            if marker.id == self._aruco_id:
                print('Received aruco pose, marker ID', marker.id)
                self._aruco_pose = marker.pose
                if not self.grasping:
                    self.counter += 1

        if self.counter > 6:
            self._aruco_found = True

    def send_goal(self, pose_goal):

        # Look around (To build a full octomap for obstacle avoidance with arm)
        # head_control = LookToPoint()
        # point = geometry_msgs.msg.Point()
        # point.x = 1.0
        # point.y = 0.0
        # point.z = 0.0
        # head_control.run(point)

        # Wait until aruco marker is found (program can get stuck here!)
        while not self._aruco_found:
            rospy.loginfo('Looking for aruco!')
            rospy.sleep(0.1)

        rospy.loginfo('Confident on aruco!')
        # Reset aruco counter and specify you are in grasping
        self.counter = 0
        self.grasping = True
        self._aruco_found = False

        # Populate the grasp and pre grasp poses from aruco
        self.pose_place.position = copy.deepcopy(self._aruco_pose.pose.position)
        self.pose_preplace.position = copy.deepcopy(self._aruco_pose.pose.position)

        # Correct for frame gripper and object grasp point (aruco is on top)
        self.pose_place.position.z += 0.21
        self.pose_preplace.position.z += 0.21

        self.pose_place.position.x -= 0.16
        self.pose_preplace.position.x -= 0.30


        # Add the routine to look around
        ########
        # Add here + check if no aruco found
        ########
                
        # place routine
        self.tiago_moveit.run(self.pose_preplace)
        self.tiago_moveit.run(self.pose_place)
        self.tiago_gripper.run('open')
        self.tiago_moveit.run(self.pose_preplace)

        #print('Preplace loc', self.pose_preplace.position)
        #print('Place loc', self.pose_place.position)

        self.tiago_moveit.run(self.pose_rest)

        # Grasping terminated
        self.grasping = False

    def get_state(self):
        # To modify with the actual return status 
        return 1

    def cancel_goal(self):
        # To modify with the actual return status 
        pass
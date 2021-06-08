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
        self._aruco_id = 111
        self._aruco_found = False
        
        # Move it and gripper
        self.tiago_moveit = tiagoMoveit('right')
        self.tiago_gripper = GripperControl('right')

        self.pose_preplace = geometry_msgs.msg.Pose()
        self.pose_rest = geometry_msgs.msg.Pose()
        self.pose_place = geometry_msgs.msg.Pose()
        
        self.pose_rest.position.x = 0.2
        self.pose_rest.position.y = -0.3
        self.pose_rest.position.z = 0.7
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
                #print('Received aruco pose, marker ID', marker.id)
                self._aruco_pose = marker.pose
                if not self.grasping:
                    self.counter += 1

        if self.counter > 20:
            self._aruco_found = True

    def send_goal(self, pose_goal):
        
        # Wait until aruco marker is found (program can get stuck here!)
        while not self._aruco_found:
            rospy.loginfo('Looking for aruco!')
            rospy.sleep(0.1)

        rospy.loginfo('Confident on aruco!')
        # Reset aruco counter and specify you are in grasping
        self.counter = 0
        self.grasping = True
        self._aruco_found = False

        # Add shelf collision object
        shelf_pose = geometry_msgs.msg.PoseStamped()
        shelf_pose.header.frame_id = "map"
        shelf_pose.pose.orientation.z = 1.0
        shelf_pose.pose.position.x = 0.45
        shelf_pose.pose.position.y = -2.2
        shelf_pose.pose.position.z = 0.3
        box_name = "shelf"
        self.tiago_moveit.scene.add_box(box_name, shelf_pose, size=(1.0, 0.8, 0.76))

        # Populate the grasp and pre grasp poses from aruco
        self.pose_place.position = copy.deepcopy(self._aruco_pose.pose.position)
        self.pose_preplace.position = copy.deepcopy(self._aruco_pose.pose.position)

        # Correct for frame gripper and object grasp point (aruco is on top)
        self.pose_place.position.z += 0.15
        self.pose_preplace.position.z += 0.21

        self.pose_place.position.x -= 0.20
        self.pose_preplace.position.x -= 0.20

        print('pre-place pose is', self.pose_preplace)    
        # place routine
        self.tiago_moveit.run(self.pose_preplace)
        self.tiago_moveit.run(self.pose_place)
        self.tiago_gripper.run('open')
        self.tiago_moveit.run(self.pose_preplace)

        self.tiago_moveit.run(self.pose_rest)

        # Grasping terminated
        self.grasping = False

    def get_state(self):
        # To modify with the actual return status 
        return 1

    def cancel_goal(self):
        # To modify with the actual return status 
        pass
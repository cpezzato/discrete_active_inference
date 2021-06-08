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
import numpy as np
from sensor_msgs.msg import JointState

class tiagoPick(object):
    def __init__(self):

        # Aruco
        self._aruco_pose = geometry_msgs.msg.PoseStamped()
        self._aruco_id = 333
        self._aruco_found = False
        
        # Finger states
        self.fingers_state = []

        # Move it and gripper
        self.tiago_moveit = tiagoMoveit('right')
        self.tiago_gripper = GripperControl('right')

        self.pose_prepick = geometry_msgs.msg.Pose()
        self.pose_rest = geometry_msgs.msg.Pose()
        self.pose_pick = geometry_msgs.msg.Pose()
        
        self.pose_rest.position.x = 0.2
        self.pose_rest.position.y = -0.3
        self.pose_rest.position.z = 0.7
        self.pose_rest.orientation.x = 0.707
        self.pose_rest.orientation.y = 0.0
        self.pose_rest.orientation.z = 0.0
        self.pose_rest.orientation.w = 0.707

        # Fixed orientation for grasp and pre grasp
        self.pose_pick.orientation = copy.deepcopy(self.pose_rest.orientation)
        self.pose_prepick.orientation = copy.deepcopy(self.pose_rest.orientation)

        # Initial pose is the rest pose
        self._aruco_pose.pose.position = copy.deepcopy(self.pose_rest.position)
        self._aruco_pose.pose.orientation = copy.deepcopy(self.pose_rest.orientation)

        # Variables for better marker detection
        self.counter = 0    
        self.grasping = False

        # Aruco sub
        self._aruco_sub = rospy.Subscriber("aruco_marker_publisher/markers", MarkerArray, self.aruco_cb)
        # Joint states
        self.gripper_subscriber = rospy.Subscriber("/joint_states", JointState, self.gripper_callback)

    def aruco_cb(self, msg):
        # Callback to update pose of detected aruco marker on the object
        for marker in msg.markers:
            if marker.id == self._aruco_id:
                # print('Received aruco pose')
                self._aruco_pose = marker.pose
                if not self.grasping:
                    self.counter += 1
        # Use 6 for real robot
        if self.counter > 20:
            self._aruco_found = True

    def gripper_callback(self, data):
        # right arm for holding stuff
        self.fingers_state = [data.position[16], data.position[17]]

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
        self.pose_pick.position = copy.deepcopy(self._aruco_pose.pose.position)
        # Correct for frame gripper and object grasp point (aruco is on top)
        self.pose_pick.position.x -= 0.22
        self.pose_pick.position.z -= 0.015
        self.pose_pick.position.y += 0.02 # aruco detection still not fixed by PAL Robotics
        self.pose_prepick.position = copy.deepcopy(self.pose_pick.position)
        self.pose_prepick.position.z += 0.12

        # Add table collision object
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = "map"
        table_pose.pose.orientation.w = 1.0
        table_pose.pose.position.x = 2.3
        table_pose.pose.position.y = 0.024
        table_pose.pose.position.z = 0.3
        box_name = "table"
        self.tiago_moveit.scene.add_box(box_name, table_pose, size=(1.3, 1.4, 0.8))

        # Add cube collision object
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_footprint"
        box_pose.pose = copy.deepcopy(self._aruco_pose.pose)
        #rospy.loginfo("box_pose: %s", box_pose.pose)

        box_height = 0.15
        box_pose.pose.position.z -= box_height/2

        box_pose.pose.orientation.x = 0.0
        box_pose.pose.orientation.y = 0.0
        box_pose.pose.orientation.z = 0.0
        box_pose.pose.orientation.w = 1.0

        box_name = "aruco_cube"
        self.tiago_moveit.scene.add_box(box_name, box_pose, size=(0.05, 0.05, box_height))

        print('The pick pose is', self.pose_pick)
        
        # Pick routine
        self.tiago_gripper.run('open')
        self.tiago_moveit.run(self.pose_prepick)
        self.tiago_moveit.run(self.pose_pick)
        self.tiago_gripper.run('close')

        # If grasp is succesfull, attach the box and complete the routine
        if np.abs(self.fingers_state[0] + self.fingers_state[1]) < 0.065 and np.abs(self.fingers_state[0] + self.fingers_state[1])>0.02:
            # Attach collision object of box to gripper
            grasping_group = "gripper_right"
            touch_links = self.tiago_moveit.robot.get_link_names(group=grasping_group)
            eef_link = self.tiago_moveit.group.get_end_effector_link()
            self.tiago_moveit.scene.attach_box(eef_link, box_name, touch_links=touch_links)

            self.tiago_moveit.run(self.pose_prepick)
            self.tiago_moveit.run(self.pose_rest)
            self.tiago_moveit.scene.remove_attached_object(eef_link, name=box_name)

        else:
            rospy.loginfo("I am not holding the object, LOL")
            
        # Check if it is holding after all the steps and remove the attached box to retry the grasp
        # if np.abs(self.fingers_state[0] + self.fingers_state[1]) < 0.065 and np.abs(self.fingers_state[0] + self.fingers_state[1])>0.02:
        #     pass
        # else:
        #     pass
                    
        self.tiago_moveit.scene.remove_world_object('aruco_cube')
        self.tiago_moveit.scene.remove_world_object('table')

        # Grasping terminated
        self.grasping = False

    def get_state(self):
        # To modify with the actual return status 
        return 1

    def cancel_goal(self):
        # To modify with the actual return status 
        pass
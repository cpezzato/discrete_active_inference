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


class detectMarker(object):
    def __init__(self):

        # Aruco
        self._aruco_sub = rospy.Subscriber("aruco_marker_publisher/markers", MarkerArray, self.aruco_cb)
        self._aruco_pub = rospy.Publisher("/objects_poses/object", geometry_msgs.msg.Pose, queue_size=10)
        
        self._aruco_pose = geometry_msgs.msg.PoseStamped()
        self._aruco_id = 1008
        self._aruco_found = False


    def aruco_cb(self, msg):
        # Callback to update pose of detected aruco marker on the object
        for marker in msg.markers:
            if marker.id == self._aruco_id:
                pass
            self._aruco_found = True
            self._aruco_pose = marker.pose
            
            
            self._aruco_pose.pose.orientation.x = 0.707
            self._aruco_pose.pose.orientation.y = 0.0
            self._aruco_pose.pose.orientation.z = 0.0
            self._aruco_pose.pose.orientation.w = 0.707        
            #print('Ready to publish',self._aruco_pose)
            self._aruco_pub.publish(self._aruco_pose.pose)
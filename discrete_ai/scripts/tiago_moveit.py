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


class tiagoMoveit(object):
    def __init__(self, side):
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        arm_side = side

        group_name = "arm_" + arm_side + "_torso"
        self.group = moveit_commander.MoveGroupCommander(group_name)

        # We can get the name of the reference frame for this robot:
        # Gripper pose will always be relative to this frame!
        planning_frame = self.group.get_planning_frame()
        print("============ Reference frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.group.get_end_effector_link()
        print("============ End effector: %s" % self.eef_link)

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print("============ Robot Groups:", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")

    def add_collision_obj(self, obj):
        # Object
        self.box_pose = geometry_msgs.msg.PoseStamped()
        self.box_pose.header.frame_id = "base_footprint"
        self.box_pose.pose.orientation = copy.deepcopy(obj.pose.orientation)
        self.box_pose.pose.position = copy.deepcopy(obj.pose.position)
        self.box_pose.pose.position.z -= 0.035 # Subtract part of the object since the marker is on top
        self.box_name = "obj_grasped"
        self.scene.add_box(self.box_name, self.box_pose, size=(0.05, 0.15, 0.05))
        rospy.loginfo('aruco cube added to collision objects, pos: %s', self.box_pose.pose.position)
        
        # Attach collision object
        grasping_group = "gripper_right"
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.eef_link, self.box_name, touch_links=touch_links)

    def run(self, pose_goal):

        self.group.set_pose_target(pose_goal)
        plan = self.group.go(wait=True)

        # Calling `stop()` ensures that there is no residual movement
        if plan:
            self.group.stop()
            # It is always good to clear your targets after planning with poses.
            # Note: there is no equivalent function for clear_joint_value_targets()
            self.group.clear_pose_targets()
            rospy.loginfo('Goal reached')
            return True
        else:
            rospy.logfatal('Failed to find a path..')
            return False
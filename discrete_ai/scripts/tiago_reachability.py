#! /usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg


class tiagoReachable(object):
    def __init__(self, side):
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        arm_side = side

        group_name = "arm_" + arm_side + "_torso"
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.ee_pose = geometry_msgs.msg.Pose()

    def isReachable(self, pose_goal):
        
        self.ee_pose.orientation.w = 0.707
        self.ee_pose.orientation.x = 0.707
        self.ee_pose.orientation.y = 0.0
        self.ee_pose.orientation.z = 0.0
        self.ee_pose.position.x = pose_goal.position.x
        self.ee_pose.position.x -= 0.22
        self.ee_pose.position.y = pose_goal.position.y
        self.ee_pose.position.z = pose_goal.position.z

        print("the received pose goal is", self.ee_pose)
        self.group.set_pose_target(self.ee_pose)
        plan = self.group.plan()
        #plan = self.group.go(wait=True)

        #print('the plan is', len(plan.joint_trajectory.points))

        if len(plan.joint_trajectory.points)>0:
            self.group.clear_pose_targets()
            rospy.loginfo('Reachable')
            return True
        else:
            rospy.logfatal('Not reachable')
            return False
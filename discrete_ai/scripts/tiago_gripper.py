#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class GripperControl(object):
  def __init__(self, side):
    #rospy.init_node("home_gripper")
    rospy.loginfo("Waiting for gripper_controller...")

    self.client = actionlib.SimpleActionClient("gripper_" + side + "_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    self.client.wait_for_server()
    rospy.loginfo("...connected.")

    rospy.wait_for_message("joint_states", JointState)

    # Set joint names of the grippers
    self.joint_names = ["gripper_" + side + "_left_finger_joint", "gripper_" + side + "_right_finger_joint"]

  def run(self, gripper_goal):
    # Set goal positions depending on open or close grippers
    if gripper_goal == 'close':
      goal_position  = [0.0, 0.0]
    elif gripper_goal == 'open':
      goal_position = [1.0, 1.0]
    else:
      rospy.logwarn('No gripper goal state selected, defaulting to close')
      goal_position = [0.0, 0.0]

    # Create trajectory
    trajectory = JointTrajectory()
    trajectory.joint_names = self.joint_names
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = goal_position
    trajectory.points[0].velocities = [0.0 for i in self.joint_names]
    trajectory.points[0].effort = [100 for i in self.joint_names]
    trajectory.points[0].accelerations = [0.0 for i in self.joint_names]
    trajectory.points[0].time_from_start = rospy.Duration(2.0)

    # Send trajectory to action client
    if gripper_goal == 'open':
      rospy.loginfo("Opening gripper...")
    elif gripper_goal == 'close':
      rospy.loginfo("Closing gripper...")
    else:
      rospy.loginfo("No gripper goal selected.")
      return 0

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = trajectory
    goal.goal_time_tolerance = rospy.Duration(0.0)

    self.client.send_goal(goal)
    rospy.loginfo(self.client.wait_for_result(rospy.Duration(3.0)))
    
    if gripper_goal == 'open':
      rospy.loginfo("Gripper opened.")
    elif gripper_goal == 'close':
      rospy.loginfo("Gripper closed.")

  
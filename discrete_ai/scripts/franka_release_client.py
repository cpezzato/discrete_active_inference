#!/usr/bin/env python

import rospy
import actionlib
import numpy as np
import copy
from std_msgs.msg import Float64MultiArray, Float64
from franka_gripper.msg import GraspAction, GraspActionGoal, GraspGoal
from franka_gripper.msg import MoveAction, MoveActionGoal, MoveGoal

from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    Point,
    Quaternion
)

from mm_msgs.srv import MMIk
import mobile_mpc.msg


class ReleaseClientClass(object):
    def __init__(self):
        self.poseSub = rospy.Subscriber("basket_pose", PoseStamped, self.objPose_cb)
        self.pubFinger1 = rospy.Publisher("/mmrobot/panda_finger1_effort_controller/command",Float64, queue_size=10)
        self.pubFinger2 = rospy.Publisher("/mmrobot/panda_finger2_effort_controller/command",Float64, queue_size=10)
        self.mpcClient = actionlib.SimpleActionClient("mpc_server", mobile_mpc.msg.mpcAction)
        self.mpcClient.wait_for_server()
        self.graspClient = actionlib.SimpleActionClient("/franka_gripper/grasp",GraspAction)
        self.moveClient = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)
        rospy.sleep(0.2)
        self.status = 0     # 0 PENDING
                            # 1 ACTIVE
                            # 2 PREEMPTED
                            # 3 SUCCEEDED

    def objPose_cb(self, pose):
        self.current_target_pose = pose

    def move_gripper(self, motion_type):
        moveActionGoal = MoveGoal()
        graspActionGoal = GraspGoal()
        if motion_type == 'open':
            moveActionGoal.width = 0.15
            graspActionGoal.width = 0.5
            graspActionGoal.speed = 0.5
        elif motion_type == 'close':
            moveActionGoal.width = 0.001
            graspActionGoal.width = 0.0
            graspActionGoal.speed = 0.05
        else:
            print('Invalid motion type, selecting opening gripper')
        moveActionGoal.speed = 0.05
        #self.moveClient.send_goal(moveActionGoal)
        self.graspClient.send_goal(graspActionGoal)
        # rospy.sleep(3)
        # graspClient.wait_for_result()

    def goto(self, pose):
        rospy.wait_for_service('ik_computation')
        computeIk = rospy.ServiceProxy('ik_computation', MMIk)

	print('debugging pose', pose)
        resp1 = computeIk(50, pose)  # 50 samples to get a nice ik

        mpcGoalConfig = np.concatenate((np.zeros(3), np.array(resp1.config.data)))

        prefix = "armMotion"
        safetyMargin = rospy.get_param(prefix + "/safetyMargin")
        myMaxError = rospy.get_param(prefix + "/maxError")
        errorWeights = rospy.get_param(prefix + "/errorWeights")
        myMPCWeights = rospy.get_param(prefix + "/weights")
        goal = mobile_mpc.msg.mpcGoal(
            goal=Float64MultiArray(data=mpcGoalConfig),
            errorWeights=Float64MultiArray(data=errorWeights),
            maxError=Float64(data=myMaxError),
            mpcWeights=Float64MultiArray(data=myMPCWeights),
            safetyMargin=Float64(data=safetyMargin)
        )
        # Sends the goal to the action server.
        self.mpcClient.send_goal(goal)
        # Waits for the server to finish performing the action.
        self.mpcClient.wait_for_result()

        # Prints out the result of executing the action
        result = self.mpcClient.get_result()

    def send_goal(self, current_goal):
        # Set status to active
        self.status = 1
        # Move towards the place location
        placePose = self.current_target_pose
        self.goto(placePose)

        # Open gripper
        self.move_gripper('open')
        # Set status to succeeded
        self.status = 3  # Succeeded

    def cancel_goal(self):
        print("Cancel Release")
        self.gripperClient.cancel_goal()
        self.status = 2

    def get_return_status(self):
        #return self.gripperClient.get_state()
        return self.status

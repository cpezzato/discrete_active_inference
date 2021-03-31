#!/usr/bin/env python

import rospy
import actionlib                    # Imports the actionlib library used for implementing simple actions
import numpy as np
import copy
from std_msgs.msg import Float64MultiArray, Float64
from mm_msgs.msg import ObjectMsg
from franka_gripper.msg import GraspAction, GraspActionGoal, GraspGoal
from franka_gripper.msg import MoveAction, MoveActionGoal, MoveGoal
# from franka_gripper.msg import HomingAction, HomingActionGoal, HomingGoal

from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    Point,
    Quaternion
)

from mm_msgs.srv import MMIk
import mobile_mpc.msg


class PickClientClass(object):
    def __init__(self):
        self.poseSub = rospy.Subscriber("current_object", ObjectMsg, self.objPose_cb)
        self.mpcClient = actionlib.SimpleActionClient("mpc_server", mobile_mpc.msg.mpcAction)
        self.graspClient = actionlib.SimpleActionClient("franka_gripper/grasp", GraspAction)
        self.moveClient = actionlib.SimpleActionClient("franka_gripper/move", MoveAction)
        #self.homeClient = actionlib.SimpleActionClient("franka_gripper/homing", HomingAction)
        print('Waiting for servers')
        self.mpcClient.wait_for_server()
        self.moveClient.wait_for_server()
        print("Found for servers")
        self.status = 0     # 0 PENDING
                            # 1 ACTIVE
                            # 2 PREEMPTED
                            # 3 SUCCEEDED

    def objPose_cb(self, input_data):
        self.current_target_pose = input_data.pose

    def move_gripper(self, motion_type):
        #graspActionGoal = GraspGoal()
        moveActionGoal = MoveGoal()
        if motion_type == 'open':
            print('Opening gripper')
            #graspActionGoal.width = 0.17  # Set this bigger than max aperture to avoide unwanted closure
            moveActionGoal.width = 0.15
            moveActionGoal.speed = 0.5
        elif motion_type == 'close':
            # graspActionGoal.width = 0.005
            moveActionGoal.width = 0.005
            moveActionGoal.speed = 0.05
        else:
            print('Invalid motion type, selecting opening gripper')
        #graspActionGoal.epsilon.inner = 0.001
        #graspActionGoal.epsilon.outer = 0.001
        #graspActionGoal.speed = 0.05
        #graspActionGoal.force = 1.85
        #print('width', graspActionGoal.width)
        #self.graspClient.send_goal(graspActionGoal)
        self.moveClient.send_goal(moveActionGoal)
        #rospy.sleep(3)
        # graspClient.wait_for_result()

    def goto(self, pose, motion_type):
        rospy.wait_for_service('ik_computation')
        computeIk = rospy.ServiceProxy('ik_computation', MMIk)

        resp1 = computeIk(50, pose) # Number of samples 50 to get a good ik

        mpcGoalConfig = np.concatenate((np.zeros(3), np.array(resp1.config.data)))

        prefix = motion_type
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
        print('Waiting for MPC result')
        self.mpcClient.wait_for_result()
        print('MPC is done!!')
        # Prints out the result of executing the action
        result = self.mpcClient.get_result()

    def send_goal(self, current_goal):
        # Set status to active
        self.status = 1

        graspPose = self.current_target_pose
        preGraspPose = copy.deepcopy(graspPose)
        preGraspPose.pose.position.z += 0.06

        # Open gripper and attempt grasp
        self.move_gripper('open')

        self.goto(preGraspPose, "safeArmMotion")

        self.goto(graspPose, "armMotion")

        # Close gripper and lift
        self.move_gripper('close')

        self.goto(preGraspPose, "armMotion")

        # Move to rest position
        # self.goto(preGraspPose)
        # Set status to succeeded
        self.status = 3  # Succeeded
        

    def cancel_goal(self):
        mpcStatus = self.mpcClient.get_state()
        self.mpcClient.cancel_goal()
        self.status = 2

    def get_return_status(self):
        return self.mpcClient.get_state()
        #return self.status



#!/usr/bin/env python

import rospy
import actionlib                    # Imports the actionlib library used for implementing simple actions
from std_msgs.msg import Float64MultiArray, Float64
import numpy as np
import copy
import mobile_mpc.msg


class PushClientClass(object):
    # Class constructor
    def __init__(self):
        self.mpcClient = actionlib.SimpleActionClient("mpc_server", mobile_mpc.msg.mpcAction)
        print('Waiting for servers')
        self.mpcClient.wait_for_server()
        print("Found for servers")
        self.pre_push = [1.64, 0.07, -1.09, -2.79, 2.23, 1.67, -2.09]
        self.push = [1.4, 0.07, -1.09, -2.79, 2.23, 1.67, -2.09]
        self.rest = [-0.19, -1.13, 0.02, -2.61, 0.0, 1.48, 0]

    def goto(self, joint_values):
        oneGoal = joint_values
        prefixNumeric = 2
        if prefixNumeric == 1:
            prefix = "fineMotion"
        elif prefixNumeric == 2:
            prefix = "armMotion"
        else:
            prefix = "navigation"
        rospy.loginfo("%s" % prefix)
        safetyMargin = rospy.get_param(prefix + "/safetyMargin")
        myMaxError = rospy.get_param(prefix + "/maxError")
        errorWeights = rospy.get_param(prefix + "/errorWeights")
        myMPCWeights = rospy.get_param(prefix + "/weights")
        goal = mobile_mpc.msg.mpcGoal(
            goal=Float64MultiArray(data=oneGoal),
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
        print('Pushing')

        self.goto(self.pre_push)
        self.goto(self.push)
        self.goto(self.pre_push)
        self.goto(self.rest)

    def cancel_goal(self):
        current_status = self.get_return_status
        if current_status == 0 or current_status == 1:  # If goal is pending or active
            self.client.cancel_goal()

    def get_return_status(self):
        return self.client.get_state()

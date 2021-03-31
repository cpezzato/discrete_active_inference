#!/usr/bin/env python

import rospy
import actionlib                    # Imports the actionlib library used for implementing simple actions
from std_msgs.msg import Float64MultiArray, Float64
from behavior_control.srv import *  # Import for symbolic_perception_service
import mobile_mpc.msg


class MPCClientClass(object):
    def __init__(self):
        self.client = actionlib.SimpleActionClient("mpc_server", mobile_mpc.msg.mpcAction)
        # Add status to return, this will be in the action servers
        self.status = 0     # 0 PENDING
                            # 1 ACTIVE
                            # 2 PREEMPTED
                            # 3 SUCCEEDED

    def send_goal(self, current_goal):
        # Set status to active
        self.status = 1

        # print('Waiting for server...')
        self.client.wait_for_server()

        oneGoal = current_goal.parameters.data[:-2]
        prefixNumeric = current_goal.parameters.data[-2]
        if prefixNumeric ==1:
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
        self.client.send_goal(goal)
	print('Sent goal without waiting for result')
        # TODO to be removed and use instead the action sever status: do not send new goal if status is running.
        #self.client.wait_for_result()
        # Set status to succeeded
        self.status = 3  # Succeeded

    def cancel_goal(self):
        self.client.cancel_goal()
        self.status = 2

    def get_return_status(self):
        return self.client.get_state()
        #return self.status

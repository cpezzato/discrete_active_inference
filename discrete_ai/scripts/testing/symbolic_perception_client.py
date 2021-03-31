#!/usr/bin/env python

import sys
import rospy
from behavior_control.srv import *
from std_msgs.msg import Float64MultiArray
import numpy as np


def symbolic_perception_client(state_index, parameters):
    rospy.wait_for_service('symbolic_perception')
    try:
        current_belief = rospy.ServiceProxy('symbolic_perception', symbolicPerception)
        response = current_belief(state_index, parameters, 0)
        return response.belief_d.data, response.estimated_state
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    state_index = 0
    parameters = Float64MultiArray()
    parameters.data = np.array([0.005, 0.])
    result_belief, result_state = symbolic_perception_client(state_index, parameters)
    # print("State index and parameters %s+%s"%(state_index, parameters))
    # print('Results', result_belief, result_state)
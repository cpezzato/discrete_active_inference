#! /usr/bin/env python
from __future__ import print_function
import rospy
import actionlib
import behavior_control.msg
from std_msgs.msg import Float64MultiArray, Float64
import numpy as np


def aipbt_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (AIPBTAction) to the constructor.
    client = actionlib.SimpleActionClient('AIPBT_server', behavior_control.msg.AIPBTAction)

    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo('Waiting for server...')
    client.wait_for_server()

    # Creates a goal to send to the action server.
    par = Float64MultiArray()
    par.data = np.array([2.8, 0., 0.8])
    goal = behavior_control.msg.AIPBTGoal(prior=0, state_index=0, parameters=par) # Set for state 0 the prior on the first state so 0

    # Sends the goal to the action server.
    client.send_goal(goal)
    rospy.loginfo('Goal sent!')

    # Waits for the server to finish performing the action.
    client.wait_for_result()
    # Prints out the result of executing the action
    return client.get_state()


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('AIPBT_client_py')
        result = aipbt_client()
        print("Result:", result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)

#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import numpy as np


def gripper_callback(input_data):
    global fingers_state
    global obj_size
    global holding_threshold
    fingers_state = [input_data.position[10], input_data.position[11]]
    print('Fingers state is: ',fingers_state)
    if np.abs(fingers_state[0] + fingers_state[1] - obj_size) < holding_threshold:  # Less than 6cm
	print('YES')        
    else:
        print('NO')

if __name__ == "__main__":
    rospy.init_node('symbolic_perception_server')
    gripper_subscriber = rospy.Subscriber("/joint_states", JointState, gripper_callback)
    print("Gripper subscriber is ready") 
    obj_size = 0.032
    holding_threshold = 0.005
    fingers_state = []
    rospy.spin()

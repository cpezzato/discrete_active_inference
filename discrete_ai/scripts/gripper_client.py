#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64


class GripperClientClass(object):
    # Class constructor
    def __init__(self):
        # Initialize
        self.pubFinger1 = rospy.Publisher("/mmrobot/panda_finger1_effort_controller/command", Float64, queue_size=10)
        self.pubFinger2 = rospy.Publisher("/mmrobot/panda_finger2_effort_controller/command", Float64, queue_size=10)
        self.openEffort = Float64(data=1.0)
        self.closeEffort = Float64(data=-1.0)
        rospy.sleep(0.5)

    def moveGripperEffort(self, motion_type):
        if motion_type == "close":
            self.pubFinger1.publish(self.closeEffort)
            self.pubFinger2.publish(self.closeEffort)
        elif motion_type == "open":
            self.pubFinger1.publish(self.openEffort)
            self.pubFinger2.publish(self.openEffort)
        else:
            print('Invalid input, type is either close or open!')

    def cancel_goal(self):
        self.client.cancel_goal()
        # print('Requested to cancel goal to the arm')

    def get_return_status(self):
        return self.client.get_state()

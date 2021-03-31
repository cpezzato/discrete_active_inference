#!/usr/bin/env python

import rospy
import actionlib                    # Imports the actionlib library used for implementing simple actions
import numpy as np
import copy
from std_msgs.msg import Float64MultiArray, Float64
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from mm_msgs.msg import ObjectMsg

from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    Point,
    Quaternion
)

from mm_msgs.srv import MMIk
import mobile_mpc.msg


class GraspClientClass(object):
    def __init__(self):
        self.poseSub = rospy.Subscriber("current_object", ObjectMsg, self.objPose_cb)
        self.pubFinger1 = rospy.Publisher("/mmrobot/panda_finger1_effort_controller/command",Float64, queue_size=10)
        self.pubFinger2 = rospy.Publisher("/mmrobot/panda_finger2_effort_controller/command",Float64, queue_size=10)
        self.mpcClient = actionlib.SimpleActionClient("mpc_server", mobile_mpc.msg.mpcAction)
        #self.gripperClient = actionlib.SimpleActionClient("/mmrobot/panda_hand_controller/follow_joint_trajectory",FollowJointTrajectoryAction)
        #self.gripperClient.wait_for_server()
        self.mpcClient.wait_for_server()
        rospy.sleep(0.2)

    def objPose_cb(self, input_data):
        self.current_target_pose = input_data.pose

    def move_gripper(self, config):

        actionGoal = FollowJointTrajectoryGoal()
        goal = JointTrajectory()
        goal.joint_names = ['mmrobot_finger_joint1', 'mmrobot_finger_joint2']
        goal.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.2)
        goalPoint = JointTrajectoryPoint()
        goalPoint.positions = config
        goalPoint.effort= [0.05, 0.05]
        goalPoint.time_from_start= rospy.Duration.from_sec(1.0)
        goal.points.append(goalPoint)
        actionGoal.trajectory = goal

        self.gripperClient.send_goal(actionGoal)
        self.gripperClient.wait_for_result()

    def move_gripper_effort(self, effort):
        self.pubFinger1.publish(effort)
        self.pubFinger2.publish(effort)

    def close_gripper(self):
        #self.move_gripper([0.0, 0.0])
        self.move_gripper_effort(Float64(data=-1.0))


    def open_gripper(self):
        #self.move_gripper([0.04, 0.04])
        self.move_gripper_effort(Float64(data=1.0))

    def goto(self, pose):
        rospy.wait_for_service('ik_computation')
        computeIk = rospy.ServiceProxy('ik_computation', MMIk)

        resp1 = computeIk(pose)

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
        graspPose = self.current_target_pose
        preGraspPose = copy.deepcopy(graspPose)
        preGraspPose.pose.position.z += 0.1
        self.goto(preGraspPose)
        self.goto(graspPose)

        # Close gripper
        self.close_gripper()

    def cancel_goal(self):
        mpcStatus = self.mpcClient.get_state()
        print(mpcStatus)
        #gripperStatus = self.gripperClient.get_state()
        #print(gripperStatus)
        self.mpcClient.cancel_goal()
        #self.gripperClient.cancel_goal()

    def get_return_status(self):
        return self.mpcClient.get_state()

#!/usr/bin/env python

from behavior_control.srv import symbolicPerception, symbolicPerceptionResponse
import rospy
from std_msgs.msg import Float64MultiArray, Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import numpy as np
import AIP                      # Module for active inference routine
import demo_templates           # Module for action templates and active inference states
from mm_msgs.srv import MMIk
from mm_msgs.msg import ObjectMsg
from geometry_msgs.msg import PoseStamped, Pose



def gripper_callback(input_data):
    global fingers_state
    fingers_state = [input_data.position[10], input_data.position[11]]


def base_callback(input_data):
    global base_location
    base_location = np.array([input_data.data[0], input_data.data[1]])

def mpcError_callback(input_data):
    global mpcError
    mpcError = input_data.data


def objPose_callback(input_data):
    global obj_pose
    global obj_size
    obj_pose = input_data.pose
    obj_size = input_data.size.data
    # print('obj_size: ', obj_size)
    # print('obj_pose: ', obj_pose)

def basketPose_callback(input_data):
    global basket_pose
    basket_pose = input_data

def handle_symbolic_perception_request(req):
    # print("Received state_index %s and parameters %s]"%(req.state_index, req.parameters.data))
    o_isHolding = []
    o_isReachable = []
    o_isAt = []


    #print("req : ", req.parameters.data)
    poseSelector = np.array(req.parameters.data[-1])
    global goal
    global mpcError
    global maxError
    global basket_pose
    global obj_size
    global obj_pose
    global holding_threshold
    global base_location

    # Set the newGoal, which is the whole parameters.data minus the last two entries (which represent the type of motion
    # and the poseSelector for either the object or the basket pose)
    newGoal = np.array(req.parameters.data[:-2])
    #print('New Goal: ', newGoal)
    if (len(newGoal) > 5) and (np.linalg.norm(newGoal - goal) > 0.01):
        goal = newGoal
        mpcError = 10.0

    # Perform manipulation of the sensed states to determine the symbolic observation
    # The order of the states is the following [mdp_h, mdp_r, mdp_l] s isHolding, isReachable, isAt
    if not fingers_state:
        print('Missing finger states, check the relevant topics are published')
    elif req.state_index < n_mdps:  # If state_index is valid populate observations
        if req.state_index == 0:  # Check if it is holding according to fingers' aperture
	    #print('obj_size:', obj_size)
            if np.abs(fingers_state[0] + fingers_state[1] - obj_size) < holding_threshold:  # Less than 6cm
                print('Is holding')
                o_isHolding = 0
                # Setting observation
                setattr(mdp_h, 'o', o_isHolding)  # 0 = Holding, 1 = notHolding
            else:
                print('Is Not holding')
                o_isHolding = 1
                # Setting observation
                setattr(mdp_h, 'o', o_isHolding)  # 0 = Holding, 1 = notHolding
        if req.state_index == 1:  # Check if it is reachable according to base pose (to be changed with reachability)
            # rospy.wait_for_service('ik_computation')
            computeIk = rospy.ServiceProxy('ik_computation', MMIk)

            # Selecting what pose is required by the current goal state
            ikErrorFlag = -1  # Set error flag to -1, meaning the IK service has not been used yet
            if poseSelector == 0:  # This 0 or 1 are set in the demo.xml file while composing the behavior
                targetPose = obj_pose  # Retrieve target pose from the obj_pose, which is subscribing to the topic
                ikResp = computeIk(10, targetPose)  # Get ikResponse and set the error flag
                ikErrorFlag = ikResp.errorFlag
            elif poseSelector == 1:
                targetPose = basket_pose
                ikResp = computeIk(10, targetPose)
                ikErrorFlag = ikResp.errorFlag

            # Debugging IK
            print('IK error flag: ', ikErrorFlag)

            if ikErrorFlag >= 0:
                print('Is reachable')
                o_isReachable = 0
                # Setting observation
                setattr(mdp_r, 'o', o_isReachable)  # 0 = Holding, 1 = notHolding
            else:
                print('Is Not reachable')
                o_isReachable = 1
                # Setting observation
                setattr(mdp_r, 'o', o_isReachable)  # 0 = Reachable, 1 = notReachable
        if req.state_index == 2:  # Check if the base is at a specific location with tolerance
            # distError = mpcError
            # rospy.loginfo(mpcError)
	    #if distError < maxError:
	    print('Base location: ', base_location)
            print('Goal location: ', goal[:2])
	    computed_norm = np.linalg.norm(base_location-np.array([goal[0], goal[1]]))
	    print('The error norm is', computed_norm)
            if computed_norm < maxError:
                rospy.loginfo('Is at')
                o_isAt = 0
                # Setting observation
                setattr(mdp_l, 'o', o_isAt)  # 0 = At, 1 = notAt
            else:
                rospy.loginfo('Is Not at')
                o_isAt = 1
                # Setting observation
                setattr(mdp_l, 'o', o_isAt)  # 0 = At, 1 = notAt

        if req.state_index == 3:  # Check if it hand is empty
            if paper_box_location[1] > 0.9:  # To be changed with a better metric, now it is application specific
                print('Place location is free')
                o_isSpotEmpty = 0
                # Setting observation
                setattr(mdp_e, 'o', o_isSpotEmpty)  # 0 = Empty
            else:
                print('Place location is NOT free')
                o_isSpotEmpty = 1
                # Setting observation
                setattr(mdp_e, 'o', o_isSpotEmpty)  # 1 = notEmpty

        if req.state_index == 4:  # Check if obj is placed at the location
            if np.linalg.norm(np.array(box_location)-np.array(desired_place_loc)) < 0.15:  # To be changed with a better metric, now it is application specific
                print('Obj is placed at loc')
                o_isPlacedAt = 0
                # Setting observation
                setattr(mdp_p, 'o', o_isPlacedAt)
            else:
                print('Obj is NOT placed at loc')
                o_isPlacedAt = 1
                # Setting observation
                setattr(mdp_p, 'o', o_isPlacedAt)

        # Run active inference to build probabilistic belief for this state_index
        # If a new goal has been reached we need to reset the belief, otherwise we could have spurious decisions since
        # we believe things that are still related to the previous scenario
        if req.isNew == 1:
            for mdp in range(n_mdps):
                all_mdp[mdp].d = np.array([[0.5], [0.5]])

        all_mdp[req.state_index] = AIP.aip_select_action(all_mdp[req.state_index])  # I could write a simplified version
        # of AIP only for state estimation, for now it is okay like this
    else:
        print("Invalid state index")

    # Return the observation
    # server_response = [o_isHolding, o_isReachable, o_isAt]
    # print('Joint positions', fingers_state)
    # print('Base location', base_location)
    if not fingers_state:
        # print('Service returns error flag -1, empty observation')
        return_empty_belief = Float64MultiArray()
        return_empty_state = -1
        return symbolicPerceptionResponse(return_empty_belief, return_empty_state)
    else:
        # Return message, estimated initial state and most probable state
        belief_d = Float64MultiArray()
        belief_d.data = all_mdp[req.state_index].d
        state = all_mdp[req.state_index].s
        return symbolicPerceptionResponse(belief_d, state)


def symbolic_perception_server():
    rospy.init_node('symbolic_perception_server')
    perception_service = rospy.Service('symbolic_perception', symbolicPerception, handle_symbolic_perception_request)
    print("Perception server is ready")
    gripper_subscriber = rospy.Subscriber("/joint_states", JointState, gripper_callback)
    print("Gripper subscriber is ready")
    base_subscriber = rospy.Subscriber("/mmrobot/curState", Float64MultiArray, base_callback)
    print("Base subscriber is ready")
    mpcError_subscriber = rospy.Subscriber("/mpcError", Float64, mpcError_callback)
    print("MPC error subscriber is ready")
    objectPose_subscriber = rospy.Subscriber("/current_object", ObjectMsg, objPose_callback)
    print("Object pose subscriber is ready")
    basketPose_subscriber = rospy.Subscriber("/basket_pose", PoseStamped, basketPose_callback)
    print("Basket pose subscriber is ready")
    rospy.spin()


if __name__ == "__main__":
    maxError = rospy.get_param("mpc/max_error")
    #maxError = 1
    # Initialize the goal to something high, it is going to be updated later when a service call comes in
    mpcError = 10
    # Initialize goal to something not realistic, only for the first trial
    goal = np.ones(10) * 10000000
    # Initialize object parameters to something not realistic, only for the first trial
    # obj_size = rospy.get_param("grasp/object_size")
    #obj_size = 0.035
    obj_pose = PoseStamped()
    #obj_pose.pose.orientation.w = 1
    #obj_pose.pose.position.x = 100
    #obj_pose.header.frame_id = "odom"
    basket_pose = PoseStamped()
    holding_threshold = rospy.get_param("symbolic_perception/holding_threshold")
    #holding_threshold = 0.005
    fingers_state = []
    base_location = np.zeros(2)
    paper_box_location = []
    box_location = [-100, -100, -100]
    p = Pose()

    desired_place_loc = [3.883, 0.838721, 0.775]

    # 3 states, 6 total (3 for perception only and 3 for action only)
    mdp_h = demo_templates.MDPIsHolding('isHolding_se')  # State for active inference routines, state_estimation
    mdp_r = demo_templates.MDPIsReachable('isReachable_se')
    mdp_l = demo_templates.MDPIsAt('isAt_se')
    mdp_e = demo_templates.MDPIsSpotEmpty('isSpotEmpty_se')
    mdp_p = demo_templates.MDPIsPlacedAt('IsPlacedAt_se')

    # Variables containing all the states for handier loops
    all_mdp = [mdp_h, mdp_r, mdp_l, mdp_e, mdp_p]

    # State indexes, names
    state_names = ['isHolding', 'isReachable', 'isAt', 'isSpotEmpty', 'isPlacedAt']

    # Number of mdps
    n_mdps = len(all_mdp)

    # Call the service and spin
    symbolic_perception_server()

#!/usr/bin/env python

from discrete_ai.srv import symbolicPerception, symbolicPerceptionResponse
import rospy
from math import pi
from std_msgs.msg import Float64MultiArray, Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
import numpy as np
import AIP                      # Module for active inference routine
import demo_templates           # Module for action templates and active inference states
from marker_detection import detectMarker
from tiago_reachability import tiagoReachable

def gripper_callback(data):
    global fingers_state
    # Left arm for holding stuff
    fingers_state = [data.position[16], data.position[17]]
    #print('Fingers state', fingers_state)

def base_callback(data):
    global base_location

    base_location = [data.pose.pose.position.x, data.pose.pose.position.y]
    #print('Base location', base_location)

def gazebo_callback(data):
    global paper_box_location
    global box_location

    paper_box_location = [data.pose[-1].position.x, data.pose[-1].position.y, data.pose[-1].position.z]
    box_location = [data.pose[-2].position.x, data.pose[-2].position.y, data.pose[-2].position.z]
    #print('y loc', box_location[1])

def paper_box_pose_callback(data):
    global paper_box_location
    paper_box_location = [data.position.x, data.position.y, data.position.z]

def object_pose_callback(data):
    global box_location
    global box_pose 

    # Compensate for gripper length
    box_location =  [data.position.x, data.position.y, data.position.z]
    box_pose = data
    print('Box Location', box_location)
    #print('Reachable Flag', reachableFlag)

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]


def handle_symbolic_perception_request(req):
    # print("Received state_index %s and parameters %s]"%(req.state_index, req.parameters.data))
    o_isHolding = []
    o_isReachable = []
    o_isAt = []
    global tiago_reach

    # rospy.wait_for_service('ik_computation')
    #computeIk = rospy.ServiceProxy('ik_computation', MMIk)

    # Selecting what pose is required by the current goal state
    ikErrorFlag = -1  # Set error flag to -1, meaning the IK service has not been used yet

    print("Base location", base_location)
    print("Box location", box_location)

    # Perform manipulation of the sensed states to determine the symbolic observation
    # The order of the states is the following [mdp_h, mdp_r, mdp_l]
    if not fingers_state or not base_location:
        print('Missing sensory input, check the relevant topics are published')
    elif req.state_index < n_mdps:  # If state_index is valid populate observations
        if req.state_index == 0:  # Check if it is holding according to fingers' aperture
            #if np.abs(fingers_state[0] + fingers_state[1]-0.06) < 0.01:  # Less than 6cm
            if np.abs(fingers_state[0] + fingers_state[1]) < 0.065 and np.abs(fingers_state[0] + fingers_state[1])>0.02:
                print('Is holding')
                o_isHolding = 0
                # Setting observation
                setattr(mdp_h, 'o', o_isHolding)  # 0 = Holding, 1 = notHolding
            else:
                print('Is Not holding')
                o_isHolding = 1
                # Setting observation
                setattr(mdp_h, 'o', o_isHolding)  # 0 = Holding, 1 = notHolding
        if req.state_index == 1:  # Check if it is reachable according to base position (to be changed with reachability)
            # diff_dist = [base_location[0] - req.parameters.data[0], base_location[1] - req.parameters.data[1]]
            
            reachableFlag = tiago_reach.isReachable(box_pose)

            # if np.linalg.norm(diff_dist) < 0.07:  # This value will need to be changed with a more accurate calculation of the reachability
            #     ikErrorFlag = 1
            # else:
            #     ikErrorFlag = -1

            # To uncomment for tiago
            #ikResp = computeIk(pS)
            #ikErrorFlag = ikResp.errorFlag

            #if ikErrorFlag >= 0:
            if reachableFlag:
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
            diff_dist = [base_location[0] - req.parameters.data[0], base_location[1] - req.parameters.data[1]]
            # TODO change to a more reliable way of quering the symbolic state, you should not be quetying all the states at all the time since you can have different goals
            #diff_dist = [base_location[0] - 3.7, base_location[1] - 0]
            print('Base Location', base_location)
            #print('Goal Location [3.7, 0]')
            print('Diff dist', diff_dist)
            if np.linalg.norm(diff_dist) < 0.15:
                print('Is at')
                o_isAt = 0
                # Setting observation
                setattr(mdp_l, 'o', o_isAt)  # 0 = At, 1 = notAt
            else:
                print('Is Not at')
                o_isAt = 1
                # Setting observation
                setattr(mdp_l, 'o', o_isAt)  # 0 = At, 1 = notAt
            
        if req.state_index == 3:  # Check if place location is empty
            # if paper_box_location[1] > 0.9:  # To be changed with a better metric, now it is application specific
            #     print('Place location is free')
            o_isSpotEmpty = 0
            #     # Setting observation
            setattr(mdp_e, 'o', o_isSpotEmpty)  # 0 = Empty
            # else:
            #     print('Place location is NOT free')
            #     o_isSpotEmpty = 1
            #     # Setting observation
            #     setattr(mdp_e, 'o', o_isSpotEmpty)  # 1 = notEmpty
            
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
        # If a new goal has been reached we need to reset the belief, otherwise we could have spurious decisions since we believe things that are still related to the previous scenarion
        if req.isNew == 1:
            for mdp in range(n_mdps):
                all_mdp[mdp].d = np.array([[0.5], [0.5]])

        all_mdp[req.state_index] = AIP.aip_select_action(all_mdp[req.state_index])  # I could write a simplified version of AIP only for
        # state estimation, for now it is okay like this
    else:
        print("Invalid state index")

    # Return the observation
    server_response = [o_isHolding, o_isReachable, o_isAt]  # When adding active inference we can substitute the states here
    # print('Joint positions', fingers_state)
    # print('Base location', base_location)
    if not fingers_state or not base_location:
        print('Service returns error flag -1, empty observation')
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
    base_subscriber = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, base_callback)
    print("Base subscriber is ready")
    gazebo_model_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_callback)
    # Adding subscriber for object poses coming from aruco markers in the real world
    object_pose_subscriber = rospy.Subscriber("/objects_poses/object", Pose, object_pose_callback)
    paper_box_pose_subscriber = rospy.Subscriber("/objects_poses/paper_box", Pose, paper_box_pose_callback)
    rospy.spin()


if __name__ == "__main__":
    fingers_state = []
    base_location = []
    paper_box_location = []
    box_location = [-100, -100, -100]
    box_pose =  Pose()
    box_pose.position.x = 100
    box_pose.position.y = 100
    box_pose.position.z = 100

    p = Pose()
    box_detection = detectMarker()
    tiago_reach = tiagoReachable('right')

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

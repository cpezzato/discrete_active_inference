#!/usr/bin/env python

import rospy
import actionlib                    # Imports the actionlib library used for implementing simple actions
import numpy as np
from math import pi
import copy
import AIP                          # Module for active inference routine
import demo_templates               # Module for action templates and active inference states
import behavior_control.msg         # Imports the custom generated messages
from std_msgs.msg import Float64

from behavior_control.srv import *  # Import for symbolic_perception_service
# from move_base_skill_moveit import MoveBaseClientClass
# from pick_skill_moveit import PickClientClass
# from place_on_plate_skill_moveit import PlaceOnPlateClientClass
# from push_skill_moveit import PushClientClass
# from place_skill_moveit import PlaceClientClass
# from move_skill_moveit import MoveBaseDemoClientClass
#from gripper_client import GripperClientClass
from tiago_move_skill import MoveBaseTiagoClientClass
from tiago_pick_skill import tiagoPick
from tiago_place_skill import tiagoPlace
from tiago_push_skill import tiagoPush

# Service client for symbolic perception service, from which we retrieve mdp.d[] and mdp.s
def symbolic_perception_client(state_index, parameters, isNew):
    # print('Waiting for symbolic perception server')
    rospy.wait_for_service('symbolic_perception')
    try:
        current_belief = rospy.ServiceProxy('symbolic_perception', symbolicPerception)
        response = current_belief(state_index, parameters, isNew)
        return response.belief_d.data, response.estimated_state
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


# Helper function to keep callback function in AIPBTAction cleaner
def adaptive_action_selection(goal, _new_prior):
    # Init state names
    state_names = ['isHolding', 'isReachable', 'isAt', 'isSpotEmpty', 'isPlacedAt']

    # This is used to keep track of the status of the adaptive algorithm. -1 means not succeeded nor failed
    return_status = -1
    # This is what should happen every time we get a tick from the BT
    # 1. Retrieve current state (This will be from an external node or service) i.e. all_MDP = getState()

    # Get state and belief_d from symbolic_perception_service
    for mdp in range(n_mdps):
        result_belief, result_state = symbolic_perception_client(mdp, goal.parameters, _new_prior)
        all_mdp[mdp].d[0] = result_belief[0]
        all_mdp[mdp].d[1] = result_belief[1]
        all_mdp[mdp].s = result_state

    # Within the action selection routine we want to keep memory of the measured states since the prior of interest is not changed
    _new_prior = 0  # 0 means that the prior is not new, so we do not need to reset

    # Set initial belief for action selection
    for MDP in range(n_mdps):
        all_MDP[MDP].D = all_mdp[MDP].d
        # print('Initial state index %s value %s'%(MDP, all_MDP[MDP].D))

    #  At each new iteration (or tick from the BT), restore all available actions
    for MDP in range(n_mdps):
        all_MDP[MDP].set_default_preferences()
    # YOU CAN MERGE THE 3 FOR LOOPS ABOVE!

    # Check if previously pushed priors have been satisfied. If so, remove their preconditions
    # For each prior == 2, if the current desired state is met set that value to 0
    for MDP in range(n_mdps):  # Loop over different MDPs
        for index in range(2):  # Loop over values in the prior
            if all_MDP[MDP].C[index] == 2 and index == all_mdp[MDP].s:
                # Remove precondition pushed since met
                all_MDP[MDP].C[index] = 0

    # Flag to see if there are high priority preferences
    isHighPriority = 0
    for mdp in range(n_mdps):
        # Only reset if it is 1, we want to keep pushed preconditions
        for index in range(2):
            if all_MDP[mdp].C[index] > 1:
                isHighPriority = 1

    # Load the precondition first so resolve first the high level priors and then focus on the rest. If no high priority
    # priors, go with the rest
    for mdp in range(n_mdps):
        # Only reset if it is 1, we want to keep pushed preconditions
        for index in range(2):
            if all_MDP[mdp].C[index] == 1:
                all_MDP[mdp].C[index] = 0

    if isHighPriority == 0:
        # Assign the prior coming from the goal in the BT
        all_MDP[goal.state_index].C[goal.prior] = 1

    for mdp in range(n_mdps):
        print("State", state_names[mdp], "C = ", all_MDP[mdp].C)

    # Performing active inference only on MDPs with active prior (if elements in .C are > 0)
    # Initialize variable to contain the selected actions from the active mdps
    temp_u = []

    # Run active inference for action selection only for active priors
    for MDP in range(n_mdps):
        if np.max(all_MDP[MDP].C) > 0:
            all_MDP[MDP] = AIP.aip_select_action(all_MDP[MDP])
            temp_u.append(int(all_MDP[MDP].u))
            # print('Action at index %s is %s' % (MDP, all_MDP[MDP].u))

    # If none of the above active inference loops ran, since no prior is given, set idle action
    if not temp_u:
        temp_u = 0

    # If all the values in temp_u are the "Idle action" it means we reached the desired prior. SUCCESS
    if np.max(temp_u) == 0 and np.min(temp_u) == 0:
        # print('Success, all priors are satisfied')
        # Flag to check if an action has been found, or none is necessary
        _actionFound = 1
        selected_action = 0  # Idle action
        return_status = 3  # This indicates that active inference was able to satisfy the prior from the BT
    else:
        _actionFound = 0

    # Reactive loop for unforeseen events. Checking preconditions and pushing priors
    # For every "non-Idle" action, we need to check preconditions.
    # If preconditions for an action that we want to execute are missing, we push higher priority priors
    while _actionFound == 0:
        for MDP in range(n_mdps):
            if hasattr(all_MDP[MDP], 'u') and np.max(all_MDP[MDP].C) > 0:  # Perform only if "active state"
                if all_MDP[MDP].u != 0:
                    # Get current action
                    current_u = int(all_MDP[MDP].u)
                    # Get preconditions to be satisfied for this action
                    prec = all_MDP[MDP].preconditions[:, :, current_u]
                    # Flag for unmet preconditions
                    _unmet_prec = 0
                    # Check preconditions, for each row of prec
                    for this_row in range(np.shape(prec)[0]):
                        if prec[this_row, 0] != -1:  # Perform only if there are actually action preconditions
                            # Select type of MDP of interest and check the state against the desired precondition
                            if all_mdp[int(prec[this_row, 0])].s != prec[this_row, 1]:
                                _unmet_prec = 1  # If a necessary state is not met, we have an unmet precondition
                                # Push a prior for desired precondition to be satisfied
                                all_MDP[int(prec[this_row, 0])].C[int(prec[this_row, 1])] = 2
                                # Inhibit current action for the inner adaptation loop since missing preconditions
                                all_MDP[MDP].E[current_u] = 0
                    if _unmet_prec == 0:
                        selected_action = action_index[MDP, current_u]
                        _actionFound = 1
                        break

        # Get state and belief_d from symbolic_perception_service
        # for mdp in range(n_mdps):
        #     result_belief, result_state = symbolic_perception_client(mdp, goal.parameters, _new_prior)
        #     all_mdp[mdp].d[0] = result_belief[0]
        #     all_mdp[mdp].d[1] = result_belief[1]
        #     all_mdp[mdp].s = result_state
        #
        # # Check if previously pushed priors have been satisfied. If so, remove their preconditions
        # # For each prior == 2, if the current desired state is met set that value to 0
        # for MDP in range(n_mdps):  # Loop over different MDPs
        #     for index in range(2):  # Loop over values in the prior
        #         if all_MDP[MDP].C[index] == 2 and index == all_mdp[MDP].s:
        #             # Remove precondition pushed since met
        #             all_MDP[MDP].C[index] = 0
        #
        # # Every time we run active inference we need to check that we only run the high priority priors first, then the
        # # rest. We also need to check if there are left over priors after satisfying pushed preconditions
        # isHighPriority = 0
        # for mdp in range(n_mdps):
        #     # Only reset if it is 1, we want to keep pushed preconditions
        #     for index in range(2):
        #         if all_MDP[mdp].C[index] > 1:
        #             isHighPriority = 1
        #     print("State", state_names[mdp], "C = ", all_MDP[mdp].C)
        #
        # # This is needed
        # if isHighPriority == 0:
        #     # Assign the prior coming from the goal in the BT
        #     all_MDP[goal.state_index].C[goal.prior] = 1

        # Exploration with new pushed priors: run active inference with updated prior and allowable actions
        temp_u = []
        for MDP in range(n_mdps):
            if np.max(all_MDP[MDP].C) > 0:
                all_MDP[MDP] = AIP.aip_select_action(all_MDP[MDP])
                temp_u.append(int(all_MDP[MDP].u))

        # If none of the above active inference loops ran, since no prior is given, set idle action
        if not temp_u:
            temp_u = 0

        # If both actions are still "Idle", return failure. This means that there are no suitable
        # action to meet the necessary preconditions and achieve the desired initial prior
        if np.max(temp_u) == 0 and np.min(temp_u) == 0:
            print('FAILURE, I cannot execute any action')
            _actionFound = 1
            return_status = 4  # Meaning failure according to the action msgs
            break

    return selected_action, return_status  # Return a tuple


# Main class for action selection using active inference
class AIPBTAction(object):
    # Create messages that are used to publish feedback and result
    _feedback = behavior_control.msg.AIPBTFeedback()
    _result = behavior_control.msg.AIPBTResult()

    # Class constructor
    def __init__(self, name):
        rospy.loginfo('Server initialized...')
        self._action_name = name
        # Create SimpleActionServer
        self._as = actionlib.SimpleActionServer(self._action_name, behavior_control.msg.AIPBTAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        # Variables to keep track of changes in the selected action to cancel old goals and send new ones
        self.old_selected_action = -1
        self.old_goal_parameters = []

        # Defining the action clients for non blocking execution
        # nonblocking_place_plate_client = actionlib.SimpleActionClient('nonBlockingPlaceOnPlateAction', behavior_control.msg.AIPBTAction)
        # nonblocking_place_client = actionlib.SimpleActionClient('nonBlockingPlaceAction', behavior_control.msg.AIPBTAction)
        # nonblocking_pick_client = actionlib.SimpleActionClient('nonBlockingPickAction', behavior_control.msg.AIPBTAction)
        # nonblocking_push_client = actionlib.SimpleActionClient('nonBlockingPushAction', behavior_control.msg.AIPBTAction)

        # Create client
        # self.action1 = nonblocking_pick_client
        # self.action2 = nonblocking_place_plate_client
        # self.action5 = nonblocking_push_client
        # self.action6 = nonblocking_place_client
        # Wait for server
        # print('Waiting for skills (actions servers)')
        # self.action1.wait_for_server()
        # self.action2.wait_for_server()
        # self.action5.wait_for_server()
        # self.action6.wait_for_server()

        # Define action clients
        #self.action1 = PickClientClass()
        #self.action2 = PlaceOnPlateClientClass()
        
        # Panda
        #self.action3 = MoveBaseDemoClientClass()
        #self.action4 = MoveBaseClientClass()
        
        # Tiago
        self.action1 = tiagoPick()
        self.action3 = MoveBaseTiagoClientClass()
        self.action4 = MoveBaseTiagoClientClass()
        self.action5 = tiagoPush()
        self.action6 = tiagoPlace()
        print('Found all skills')

        self.action_clients = [self.action1, self.action6, self.action3, self.action4, self.action5, self.action6]
        #self.testAction = self.action1
        #self.action_clients = [self.testAction, self.testAction, self.testAction, self.testAction, self.testAction, self.testAction]


    def execute_cb(self, goal):
        # rospy.loginfo('Goal received...')
        state_names = ['isHolding', 'isReachable', 'isAt', 'isSpotEmpty', 'isPlacedAt']
        # rospy.loginfo('Current goal state: %s' % state_names[goal.state_index])

        # Setting flag to reset perception system for this new prior. If 0 the beliefs are re-initialized in symbolic_perception_server
        _new_prior = 1

        # This is the execute callback function that we'll run every time a new goal is received
        # r = rospy.Rate(0.5)  # Looping at 1 second
        self._feedback.errorFlag = 0

         # I can access things with goal.prior, and goal.index, both int32
        # rospy.loginfo('Received prior %i index %i' % (goal.prior, goal.state_index))
        # print('Received parameters', goal.parameters.data)

        # Loop for continuous action selection upon receiving a new prior from the BT. The loop terminates when either
        # SUCCESS or FAILURE is received by the adaptive_action_selection routine. The loop is also terminated by an
        # external halt from the BT client, or if an action that is being executed returns failure
        status_AIPBT = -1
        counter = 0
        while status_AIPBT != 3 and status_AIPBT != 4:
            # Action selection and self-adaptation loop for reactive action planning and execution
            action_to_perform, status_AIPBT = adaptive_action_selection(goal, _new_prior)
            print('Selected action: ', action_name[action_to_perform])

            # Checking selected action from active inference and cancelling or sending new goals
            if (action_to_perform == self.old_selected_action and goal.parameters != self.old_goal_parameters) or (
                    action_to_perform != self.old_selected_action):
                if self.old_selected_action > 0 and action_to_perform != 0:  # If the previous action is not idle or undefined and if the current one is idle
                    print('Cancel goal by AIP')
                    action_client.cancel_goal() # If you do not put the cancel goal here, the action will keep running until succeeded from the action server
                    # Reset here the prior of a cancelled goal
                    if self.old_selected_action == 3 and all_MDP[1].C[0] > 1:  # Move_MPC
                        all_MDP[1].C[0] = 0

                # Send the goal if action is not idle
                #print(action_to_perform)
                #print(self.action_clients)

            ## MODIFIED not TO SEND GOALS EVEN IF THEY ARE THE same
            if action_to_perform > 0:
                action_client = self.action_clients[action_to_perform-1]
                # rospy.loginfo('Sending new goal')
                action_client.send_goal(goal)
                print("Return status", action_client.get_state())
                status_current_action = action_client.get_state()

            # WHEN THE PERCEPTION NODE WILL BE AVAILABLE, status_AIPBT SHOULD BE ASSIGNED ONLY IF IT HAS FAILED OR PREEMPTED
            if action_to_perform == 0:
                status_current_action = 1

            # If the status of the action that AIP selected and that was executing is FAILURE, we exit the loop and set
            # the status of the AIPBT to FAILURE to inform the BT
            if status_current_action == 4:  # The action has failed
                status_AIPBT = 4
                break

            # Set values for previous action and parameters to be used in the next cycle
            self.old_selected_action = copy.copy(action_to_perform)
            self.old_goal_parameters = copy.copy(goal.parameters)

            # When a client requests that the current goal should be preempted:
            if self._as.is_preempt_requested():
                # You should add here the necessary clean-up and cancel the goal sent to the client
                # rospy.loginfo('%s: Preempted' % self._action_name)
                print('Cancel goal by BT')
                self._as.set_aborted()
                break

            # publish the feedback
            self._as.publish_feedback(self._feedback)
            rospy.sleep(0.02)  # Avoid super busy loops

        self.old_selected_action = -1     # Re-initialize for subsequent run if action server is still up

        # Documentation
        # https://docs.ros.org/diamondback/api/actionlib/html/classactionlib_1_1SimpleActionServer.html#a4964ef9e28f5620e87909c41f0458ecb
        # Set the success or not of the action, to be used by the client
        # Documentation: http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatus.html
        if status_AIPBT == 3:
            self._result.success = True
            #rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded()
        if status_AIPBT == 4:
            self._result.success = False
            #rospy.loginfo("%s: Aborted" % self._action_name)
            self._as.set_aborted()


if __name__ == '__main__':
    rospy.init_node('AIPBT_server')
    rospy.loginfo('Server Started...')

    # Definition of the MDP structures to be used as global variables
    # 3 states, 6 total (3 for perception only and 3 for action only)
    mdp_h = demo_templates.MDPIsHolding('isHolding_se')  # State for active inference routines, state_estimation
    mdp_r = demo_templates.MDPIsReachable('isReachable_se')
    mdp_l = demo_templates.MDPIsAt('isAt_se')
    mdp_e = demo_templates.MDPIsSpotEmpty('isSpotEmpty_se')
    mdp_p = demo_templates.MDPIsPlacedAt('IsPlacedAt_se')

    MDP_h = demo_templates.MDPIsHolding('isHolding_as')  # State for active inference routines, action_selection
    MDP_r = demo_templates.MDPIsReachable('isReachable_as')
    MDP_l = demo_templates.MDPIsAt('isAt_as')
    MDP_e = demo_templates.MDPIsSpotEmpty('isSpotEmpty_as')
    MDP_p = demo_templates.MDPIsPlacedAt('IsPlacedAt_as')

    # Remove d field for action selection
    del MDP_h.d
    del MDP_r.d
    del MDP_l.d
    del MDP_e.d
    del MDP_p.d

    # Variables containing all the states for handier loops
    all_mdp = [mdp_h, mdp_r, mdp_l, mdp_e, mdp_p]
    all_MDP = [MDP_h, MDP_r, MDP_l, MDP_e, MDP_p]

    # Number of mdps
    n_mdps = len(all_mdp)

    # Symbolic action indexes = 0: Idle, 1: Pick, 2: Place, 3: Move MPC, 4: Move move_base 5: Open_gripper
    action_index = np.array([[0, 1, 2],     # First state isHolding
                             [0, 3, 0],     # Second state isReachable
                             [0, 4, 0],     # Third state isAt
                             [0, 5, 0],     # fourth state isAHandEmpty
                             [0, 6, 0]])    # Fifth state isPlacedAt

    action_name = ['Idle', 'Pick', 'PlaceOnPlate', 'Move_MPC', 'Move_base', 'Push', 'Place']

    server = AIPBTAction(rospy.get_name())
    rospy.spin()

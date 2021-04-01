
# This is a module which contains the templates for the classes to define MDP problems to feed to active inference
# This is the pool of actions and states that we can manipulate
import numpy as np


class MDPIsHolding:
    # We need to define constructor, basically always. This needed to initialize the object
    # This is the method which is executed automatically when an object of type dg is created
    # When we define the object, we need to define the parameter 'name' since
    # when we create the object we execute the method init
    # In classes, attributes are variables that belong to a specific object
    def __init__(self, name):
        # Initialization of the attributes within this class which will be used in AIP for action selection. Here you
        # define your action templates with preconditions and actions
        self.name = name  # Name of this specific state

        self.V = np.array([0, 1, 2])  # Allowable policies, it indicates policies of depth 1
        self.B = np.zeros((2, 2, 3))  # Allowable actions initiation
        # Transition matrices
        # ----------------------------------------------------------
        self.B[:, :, 0] = np.eye(2)  # Idle action
        self.B[:, :, 1] = np.array([[1, 1],  # Pick action
                                    [0, 0]])
        self.B[:, :, 2] = np.array([[0, 0],  # Place action
                                    [1, 1]])
        # Preconditions of the actions above
        # ----------------------------------------------------------
        self.preconditions = np.zeros((2, 2, 3)) - 1	    # Action pick
        # self.preconditions[:, :, 1] = np.array([[1, 0],     # isReachable; state 1 value 0 (so it is reachable)
        #                                         [3, 0]])    # isHandFree: state 3 value 0 (so it is not holding anything) Remember indexes starts from 0

        # Only is Reachable
        self.preconditions[:, :, 1] = np.array([[1, 0],     # isReachable; state 1 value 0 (so it is reachable)
                                                [-1, -1]])

        # Likelihood matrix matrices
        # ----------------------------------------------------------
        self.A = np.eye(2)  # Identity mapping
        # Prior preferences, initially set to zero, so no preference
        # -----------------------------------------------------------
        self.C = np.array([[0.], [0.]])
        # Belief about initial state, D
        # -----------------------------------------------------------
        self.D = np.array([[0.5], [0.5]])
        # Initial guess about the states d, all equally possible, this is updated over time
        # -----------------------------------------------------------
        self.d = np.array([[0.5], [0.5]])
        # Preference about actions, idle is slightly preferred
        # -----------------------------------------------------------
        self.E = np.array([[1.01], [1], [1]])
        # Learning rate for initial state update
        # -----------------------------------------------------------
        self.kappa_d = 0.2

    def update_preferences(self, new_pref):
        self.E = new_pref

    def set_default_preferences(self):
        self.E = np.array([[1.01], [1], [1]])

    def update_initial_state(self, new_D):
        self.D = new_D

    def update_belief_initial_state(self, new_d):
        self.d = new_d

    def update_prior(self, new_C):
        self.C = new_C


class MDPIsAt:
    def __init__(self, name):
        self.name = name  # Name of this specific state

        self.V = np.array([0, 1])  # Allowable policies, it indicates policies of depth 1
        self.B = np.zeros((2, 2, 2))  # Allowable actions initiation
        # Transition matrices
        # ----------------------------------------------------------
        self.B[:, :, 0] = np.eye(2)  # Idle action
        self.B[:, :, 1] = np.array([[1, 1],  # move(loc): a_mvBase makes isAt true
                                    [0, 0]])

        # Preconditions of the actions above
        # ----------------------------------------------------------
        self.preconditions = np.zeros((2, 2, 2)) - 1  # No preconditions needed for Idle and a_mv, set to -1

        # Likelihood matrix matrices
        # ----------------------------------------------------------
        self.A = np.eye(2)  # Identity mapping
        # Prior preferences, initially set to zero, so no preference
        # -----------------------------------------------------------
        self.C = np.array([[0.], [0.]])
        # Belief about initial state, D
        # -----------------------------------------------------------
        self.D = np.array([[0.5], [0.5]])
        # Initial guess about the states d, all equally possible, this is updated over time
        # -----------------------------------------------------------
        self.d = np.array([[0.5], [0.5]])
        # Preference about actions, idle is slightly preferred
        # -----------------------------------------------------------
        self.E = np.array([[1.01], [1]])
        # Learning rate for initial state update
        # -----------------------------------------------------------
        self.kappa_d = 0.2

    def update_preferences(self, new_pref):
        self.E = new_pref

    def set_default_preferences(self):
        self.E = np.array([[1.01], [1]])

    def update_initial_state(self, new_D):
        self.D = new_D

    def update_belief_initial_state(self, new_d):
        self.d = new_d

    def update_prior(self, new_C):
        self.C = new_C


class MDPIsReachable:
    def __init__(self, name):
        self.name = name  # Name of this specific state

        self.V = np.array([0, 1])  # Allowable policies, it indicates policies of depth 1
        self.B = np.zeros((2, 2, 2))  # Allowable actions initiation
        # Transition matrices
        # ----------------------------------------------------------
        self.B[:, :, 0] = np.eye(2)  # Idle action
        self.B[:, :, 1] = np.array([[1, 1],  # move(loc): a_mvMPC makes isReachable true
                                    [0, 0]])

        # Preconditions of the actions above
        # ----------------------------------------------------------
        self.preconditions = np.zeros((2, 2, 2)) - 1  # No preconditions needed for Idle and a_mv, set to -1

        # Likelihood matrix matrices
        # ----------------------------------------------------------
        self.A = np.eye(2)  # Identity mapping
        # Prior preferences, initially set to zero, so no preference
        # -----------------------------------------------------------
        self.C = np.array([[0.], [0.]])
        # Belief about initial state, D
        # -----------------------------------------------------------
        self.D = np.array([[0.5], [0.5]])
        # Initial guess about the states d, all equally possible, this is updated over time
        # -----------------------------------------------------------
        self.d = np.array([[0.5], [0.5]])
        # Preference about actions, idle is slightly preferred
        # -----------------------------------------------------------
        self.E = np.array([[1.01], [1]])
        # Learning rate for initial state update
        # -----------------------------------------------------------
        self.kappa_d = 0.2

    def update_preferences(self, new_pref):
        self.E = new_pref

    def set_default_preferences(self):
        self.E = np.array([[1.01], [1]])

    def update_initial_state(self, new_D):
        self.D = new_D

    def update_belief_initial_state(self, new_d):
        self.d = new_d

    def update_prior(self, new_C):
        self.C = new_C


class MDPIsSpotEmpty:
    def __init__(self, name):
        # Initialization of the attributes within this class which will be used in AIP for action selection. Here you
        # define your action templates with preconditions and actions
        self.name = name  # Name of this specific state

        self.V = np.array([0, 1])  # Allowable policies, it indicates policies of depth 1
        self.B = np.zeros((2, 2, 2))  # Allowable actions initiation (idle and push)
        # Transition matrices
        # ----------------------------------------------------------
        self.B[:, :, 0] = np.eye(2)  # Idle action
        self.B[:, :, 1] = np.array([[1, 1],  # Action Push
                                    [0, 0]])
        # Preconditions of the actions above
        # ----------------------------------------------------------
        self.preconditions = np.zeros((2, 2, 2)) - 1	    # No preconditions for idle and push when using TIAGo
        
        # UNCOMMENT THIS WHEN USING SINGLE ARM SYSTEMS
        # precondition for Push is that the hand is empty
        '''self.preconditions[:, :, 1] = np.array([[0, 1],  # !isHolding; state 0 value 1 (so it is not holding anything)
                                                [-1, -1]])
        '''
        
        # Likelihood matrix matrices
        # ----------------------------------------------------------
        self.A = np.eye(2)  # Identity mapping
        # Prior preferences, initially set to zero, so no preference
        # -----------------------------------------------------------
        self.C = np.array([[0.], [0.]])
        # Belief about initial state, D
        # -----------------------------------------------------------
        self.D = np.array([[0.5], [0.5]])
        # Initial guess about the states d, all equally possible, this is updated over time
        # -----------------------------------------------------------
        self.d = np.array([[0.5], [0.5]])
        # Preference about actions, idle is slightly preferred
        # -----------------------------------------------------------
        self.E = np.array([[1.01], [1]])
        # Learning rate for initial state update
        # -----------------------------------------------------------
        self.kappa_d = 0.2

    def update_preferences(self, new_pref):
        self.E = new_pref

    def set_default_preferences(self):
        self.E = np.array([[1.01], [1]])

    def update_initial_state(self, new_D):
        self.D = new_D

    def update_belief_initial_state(self, new_d):
        self.d = new_d

    def update_prior(self, new_C):
        self.C = new_C


class MDPIsPlacedAt:
    def __init__(self, name):
        # Initialization of the attributes within this class which will be used in AIP for action selection. Here you
        # define your action templates with preconditions and actions
        self.name = name  # Name of this specific state

        self.V = np.array([0, 1])  # Allowable policies, it indicates policies of depth 1
        self.B = np.zeros((2, 2, 2))  # Allowable actions initiation (idle and place at)
        # Transition matrices
        # ----------------------------------------------------------
        self.B[:, :, 0] = np.eye(2)  # Idle action
        self.B[:, :, 1] = np.array([[1, 1],  # Action Place
                                    [0, 0]])
        # Preconditions of the actions above
        # ----------------------------------------------------------
        self.preconditions = np.zeros((2, 2, 2)) - 1	    # No preconditions foi idle
        # precondition for place at is that the hand is empty
        self.preconditions[:, :, 1] = np.array([[3, 0],  # isSpotEmpty: state 3 value 0
                                                [0, 0]])  # isHolding

        # Likelihood matrix matrices
        # ----------------------------------------------------------
        self.A = np.eye(2)  # Identity mapping
        # Prior preferences, initially set to zero, so no preference
        # -----------------------------------------------------------
        self.C = np.array([[0.], [0.]])
        # Belief about initial state, D
        # -----------------------------------------------------------
        self.D = np.array([[0.5], [0.5]])
        # Initial guess about the states d, all equally possible, this is updated over time
        # -----------------------------------------------------------
        self.d = np.array([[0.5], [0.5]])
        # Preference about actions, idle is slightly preferred
        # -----------------------------------------------------------
        self.E = np.array([[1.01], [1]])
        # Learning rate for initial state update
        # -----------------------------------------------------------
        self.kappa_d = 0.2

    def update_preferences(self, new_pref):
        self.E = new_pref

    def set_default_preferences(self):
        self.E = np.array([[1.01], [1]])

    def update_initial_state(self, new_D):
        self.D = new_D

    def update_belief_initial_state(self, new_d):
        self.d = new_d

    def update_prior(self, new_C):
        self.C = new_C
import numpy as np
import AIP  # Module for active inference routine
import demo_templates  # Module for action templates and active inference statesS
import time


def TicTocGenerator():
    # Generator that returns time differences
    ti = 0           # initial time
    tf = time.time() # final time
    while True:
        ti = tf
        tf = time.time()
        yield tf-ti # returns the time difference

TicToc = TicTocGenerator() # create an instance of the TicTocGen generator

# This will be the main function through which we define both tic() and toc()
def toc(tempBool=True):
    # Prints the time difference yielded by generator instance TicToc
    tempTimeInterval = next(TicToc)
    if tempBool:
        print( "Elapsed time: %f seconds.\n" %tempTimeInterval )

def tic():
    # Records a time in TicToc, marks the beginning of a time interval
    toc(False)

# This server is derived from the Matlab script OptimizedDemo.m
# Definition of the current states for perception and action selection
# ----------------------------------------------------------------------------------------------------------------------
mdp_h = demo_templates.MDPIsHolding('isHolding_1')           # State for active inference routines
mdp_r = demo_templates.MDPIsReachable('isReachable_1')
mdp_l = demo_templates.MDPIsAt('isAt_1')

# Setting priors
C_h = [1, 0]      # OK
C_r = [0, 0]      # OK
C_l = [0, 0]      # OK

# Setting observation
setattr(mdp_h, 'o', 0)                  # 0 = Holding, 1 = notHolding
#print('Outcome holding', mdp_h.o)
setattr(mdp_r, 'o', 0)                  # 0 = Reachable, 1 = notReachable
print('Outcome reachable', mdp_r.o)
setattr(mdp_l, 'o', 1)                  # 0 = At, 1 = notAt
# #print('Outcome at', mdp_l.o)

# Not to modify
# -------------
mdp_h.update_prior(np.array([[C_h[0]], [C_h[1]]]))
#print('Prior holding', mdp_h.C)
mdp_r.update_prior(np.array([[C_r[0]], [C_r[1]]]))
print('Prior reachable', mdp_r.C)
mdp_l.update_prior(np.array([[C_l[0]], [C_l[1]]]))
#print('Prior location', mdp_l.C)

# Active inference routines
tic()
mdp_h = AIP.aip_select_action(mdp_h)
#print('Action holding', mdp_h.u)
mdp_r = AIP.aip_select_action(mdp_r)
print('Action reachable', mdp_r.u)
mdp_l = AIP.aip_select_action(mdp_l)
toc()
print('Action at', mdp_l.u)

# REMEMBER TO SET THE INITIAL OBSERVATION: setattr(MDP_, 'addedAttribute', 3)
# Remember also that your indexes are from zero not 1


# Other testing
# ----------------------------------------------------------------------------------------------------------------------
# Definition of an array
# MDP_array = np.array([1, 0])

# # Definition of a matrix
# MDP_matrix = np.array([[1, 0], [0, 1]])

# # print(np.shape(MDP_.B)[2])
#
# MDP = np.array([[1., 0.],
#                 [1., 0.],
#                 [1., 1.]])
# # print(np.shape(MDP_vector)[0])
# # print(MDP[:, 0])
# # print(np.sum(MDP[:, 0]))
#
# MDP_vector = np.array([[1.], [2.]])
# var = AIP.aip_norm(MDP_vector)
# # print(var)
#
# softmaxArray = np.array([[2.], [3.]])
# # var = AIP.aip_softmax(softmaxArray)
# # print(var)
#
# # Multiarray with different values
# par = np.zeros((2, 2, 3))
#
# # Attributes in a class
# # print(hasattr(MDP_, 'B'))
# # print(hasattr(MDP_, 'addedAttribute'))
# # setattr(MDP_, 'addedAttribute', 3)
# # print(hasattr(MDP_, 'addedAttribute'))
# # print(MDP_.addedAttribute)
#
# # if hasattr(MDP_, 'B'):
# #     print('yeeep')
#
# #print(np.ones((2, 1)))
#
# # Get index of maximum value
# values = np.array([[0.], [0.]])
# #print(len(values))
# index = np.argmax(values)
#
# free_energy = np.zeros([3, 1])
# # print(free_energy[2])

#!/usr/bin/env python

# Simple example frm the paper illustrating active inference background

import numpy as np

# Helper functions
#--------------------------------------------------------------
def aip_log(var):
    # Natural logarithm of an element, preventing 0. The element can be a scalar, vector or matrix
    return np.log(var + np.exp(-16))

def aip_norm(var):
        # Normalisation of probability matrix (column elements sum to 1)
        # The function goes column by column and it normalise such that the
        # elements of each column sum to 1
        # In case of a matrix
        for column_id in range(np.shape(var)[1]):  # Loop over the number of columns
            sum_column = np.sum(var[:, column_id])
            if sum_column > 0:
                var[:, column_id] = var[:, column_id] / sum_column  # Divide by the sum of the column
            else:
                var[:, column_id] = 1 / np.shape(var)[0]  # Divide by the number of rows
        return var

def aip_softmax(var):
        # Function to compute the softmax of a given column array: sigma = exp(x) / sum(exp(x))
        ex = np.exp(var)  # Compute exponential
        for i in range(np.shape(var)[0]):
            var[i] = ex[i] / np.sum(ex)  # Compute softmax element by element
        return var

# Example 1 - State estimation
# -------------------------------------------------------------
A = np.array([[0.9, 0.1],
              [0.1, 0.9]])   
B = np.array([[0.8, 0.2],
              [0.2, 0.8]])  
D = np.array([[0.5], [0.5]])
o_1 = np.array([[1.], [0.]])
o_2 = np.array([[0.], [0.]])
s_t1 = np.array([[0.5], [0.5]])
s_t2 = np.array([[0.5], [0.5]])

# Update equations (see paper)
s_t1 = aip_softmax(aip_log(D) + aip_log(np.dot(np.transpose(B),s_t2)) + aip_log(np.dot(A,o_1)))
s_t2 = aip_softmax(aip_log(np.dot(B,s_t1)) + aip_log(np.dot(A,o_2)))

print('Results example 1. s_t2 is:')
print(s_t2)

# Example 2 - Exploitation 
# -------------------------------------------------------------
A = np.array([[0.9, 0.1],
              [0.1, 0.9]])    
C = np.array([[1.], [0.]])
s_pi1 = np.array([[0.95], [0.05]])
s_pi2 = np.array([[0.05], [0.95]])

# compute observations expected under the policy
o_pi1 = np.dot(A,s_pi1)
o_pi2 = np.dot(A,s_pi2)

# For G for example 4 uncomment infor gain term
G1 = np.dot(np.transpose(o_pi1), aip_log(o_pi1)-aip_log(C)) #- np.dot(np.diagonal(np.dot(np.transpose(A),aip_log(A))),s_pi1)
G2 = np.dot(np.transpose(o_pi2), aip_log(o_pi2)-aip_log(C)) #- np.dot(np.diagonal(np.dot(np.transpose(A),aip_log(A))),s_pi2)

print('Results example 2. The reward seeking term for the two plans is:')
print(G1)
print(G2)


# Example 3 - Exploration 
# -------------------------------------------------------------
A = np.array([[0.7, 0.1],
              [0.3, 0.9]])    
C = np.array([[0.], [0.]])
s_pi1 = np.array([[0.9], [0.1]])
s_pi2 = np.array([[0.1], [0.9]])

# compute observations expected under the policy

G1 = - np.dot(np.diagonal(np.dot(np.transpose(A),aip_log(A))),s_pi1) # + np.dot(np.transpose(o_pi1), aip_log(o_pi1)-aip_log(C))
G2 = - np.dot(np.diagonal(np.dot(np.transpose(A),aip_log(A))),s_pi2) # + np.dot(np.transpose(o_pi2), aip_log(o_pi2)-aip_log(C))

print('Results example 3. The reward seeking term for the two plans is:')
print(G1)
print(G2)

# Example 4 - Policy selection
# -------------------------------------------------------------
Gpi = np.array([[2.16], [13.68]])
Fpi = np.array([[1.83], [1.83]])

pi_bold = aip_softmax(-Gpi - Fpi) 
print("The posterior of policies is", pi_bold)

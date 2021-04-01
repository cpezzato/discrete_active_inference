# discrete_active_inference

Repository for active inference and behavior trees for discrete decision making. Please read the associated paper for more theorethical considerations about the algorithms.

## Current development
(Last update 01.04.2021)

This repositiry contains a Matlab example and a ros package for active inference for task planning and execution. 

### Main files 
**Matlab:**
- *aip.m* the active inference algorithm for decision making is illustrated in the case of heterogeneous states and actions. The current code shows a case with no conflicting actions, and the reactivity of the algorithm can be appreciated. The robot is unstructed to prefer to sense the door opened. In this file we only simulate what action the robot would take and we manually change the outcome just to appreciate how the agent wuld react to changing environments due to external causes. The actions are not aplied to the simulation then but only chosen to evaluate the decision making abilities. This part is implemented in *AIP_Robot_Conflicts.m*.
- *example.m* example of use of active inference for discrete decision making in a robotic case where conflicts and preconditions checks are required. A robot is assumed to be able to navigate to a point (MoveBase), reach a location with its end effector (Move MPC), and pick and place things. Actions have preconditions and are assumed not instantaneous

**ROS:**
The ROS package contains the python implementation and an example use with TIAGo (TO BE ADDED)

[1] Colledanchise, Michele, and Petter Ögren. "How behavior trees modularize hybrid control systems and generalize sequential behavior compositions, the subsumption architecture, and decision trees." IEEE Transactions on robotics 33.2 (2016): 372-389.

# discrete_active_inference

Repository for active inference and behavior trees for discrete decision making. Please read the associated paper for more theorethical considerations about the algorithms.

## Dependacies
***TIAGo++***
Install the necessary packages for TIAGo++ following the instructions at:
http://wiki.ros.org/Robots/TIAGo%2B%2B/Tutorials/Installation/Tiago%2B%2BSimulation

***Retail store simulation*** (we should release it somewhere public on github as wel)
Install packages for retail store simulation (at the current stage remove the rosplan package since we do not need it)
https://gitlab.tudelft.nl/cor/ro47014/retail_store_lightweight_sim

## Content
(Last update 01.04.2021)
This repositiry contains a Matlab example and a ros package for active inference for task planning and execution. 

### Main files 
**Matlab:**
- *aip.m* the active inference algorithm for decision making is illustrated in the case of heterogeneous states and actions. The current code shows a case with no conflicting actions, and the reactivity of the algorithm can be appreciated. The robot is unstructed to prefer to sense the door opened. In this file we only simulate what action the robot would take and we manually change the outcome just to appreciate how the agent wuld react to changing environments due to external causes. The actions are not aplied to the simulation then but only chosen to evaluate the decision making abilities. This part is implemented in *AIP_Robot_Conflicts.m*.
- *example.m* example of use of active inference for discrete decision making in a robotic case where conflicts and preconditions checks are required. A robot is assumed to be able to navigate to a point (MoveBase), reach a location with its end effector (Move MPC), and pick and place things. Actions have preconditions and are assumed not instantaneous

**ROS:**
The ROS package contains the python implementation and an example use with TIAGo (TO BE ADDED)

## How to run
- Run the simulation: $ roslaunch retail_store_simulation tiago_simulation.launch world:=multiple_cubes
- Run the active inference perception: $ rosrun discrete_ai tiago_perception.py
- Run the active inference server for deciison making: $ rosrun discrete_ai active_inference_server.py
- Run the demo: rosrun discrete_ai demo_executeBT


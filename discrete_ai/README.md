# Behavior control package - Paper on active inference


# Install package and dependencies
Install the TIAGo packages http://wiki.ros.org/Robots/TIAGo%2B%2B/Tutorials/Installation/Tiago%2B%2BSimulation. 
We will work on the paper_ai branches for the custom code

     git clone git@gitlab.tudelft.nl:mspahn/ros_mm.git
     cd ros_mm
     git checkout paper_ai
     catkin build
     git clone git@gitlab.tudelft.nl:corradopezzato/behavior_control.git
     cd behavior_control
     git checkout paper_ai
     catkin build 

# Launch the demo 
Source the WS and launch

     roslaunch mobile_manipulator mmrobot_ai.launch
     roslaunch mobile_navigation mmrobot_navigation.launch
     roslaunch mobile_moveit planning_execution.launch
     rosrun behavior_control object_spawner.py
     rosrun behavior_control symbolic_perception_server.py
     rosrun behavior_control AIPBT_server.py
     rosrun behavior_control demo_executeBT


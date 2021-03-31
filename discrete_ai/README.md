# Behavior control package

This repository provides uses the [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) library to coordinate action clients ans services using BTs and active inference. 

# Dependencies

## Behavior trees library
These are the steps you need to follow to be able to run these tutorials in Ubuntu. They have been tested in Ubuntu 18.04 with ROS Mlodic. 

Before proceeding, it is recommended to to install the following dependencies:

     sudo apt-get install libzmq3-dev libboost-dev

You can also easily install the [Behavior Tree library](https://github.com/BehaviorTree/BehaviorTree.CPP) with the command

    sudo apt-get install ros-$ROS_DISTRO-behaviortree-cpp-v3
    sudo apt-get update   

## Mobile manipulator simulation
This package is meant to be used to control the mobile manipulator in the AIRLab mockup store. The [mobile manipulator simulation](https://gitlab.tudelft.nl/mspahn/ros_mm) is required to be installed. 

# Installation

Create a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace):

     mkdir -p catkin_ws/src
     cd ..
     catkin_make

Clone the tutorials and compile:

     cd src
     git clone https://gitlab.tudelft.nl/corradopezzato/behavior_control.git
     cd ..
     catkin_make
     source devel/setup.bash

# How to use
the nodes for behavior execution can be run using

     rosrun behavior_control symbolic_perception_server.py
     rosrun behavior_control AIPBT_server.py
     rosrun behavior_control demo_executeBT

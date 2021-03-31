/*
    File: AIRLabAction_nodes.cpp

    Description: this file contains the definition of the tick methods for the action defined in AIRLabAction_nodes.h.

    Author: Corrado Pezzato, TU Delft and AIRLab
    Date last update: 31.08.2020
*/

#include "behavior_control/AIRLabAction_nodes.h"

// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time
BT_REGISTER_NODES(factory)
{
   AIRLabNodes::RegisterNodes(factory);
}

namespace AIRLabNodes
{

BT::NodeStatus ConditionisHolding::tick()
{

    // Service
      srv.request.state_index = 0;
      // Later you should define a proper port to connect to the symbolic_perception_server (state_index, parameters)
//      for (int i = 0; i < goal.parameters.data.size(); i++) {
//          msg.parameters.data.push_back(goal.parameters.data[i]);      \
//      }

      // No parameters are needed for isHolding
      //srv.request.parameters.data.push_back(-1);
      srv.request.isNew = 0;
      _srv_client.call(srv);

      if (_srv_client.call(srv))
      {
          //
          // ROS_INFO("State: %d", srv.response.estimated_state);  // Check if the condition is checked continuously
          ROS_INFO("Checking if it is holding");
      }
      else
      {
          ROS_ERROR("Failed to call service symbolic_perception");
      }

      if (srv.response.estimated_state == 0)
      {
         return BT::NodeStatus::SUCCESS;
      }
      else
         return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ConditionisReachable::tick()
{

    // Service
      srv.request.state_index = 1;
      // Later you should define a proper port to connect to the symbolic_perception_server (state_index, parameters)
//      for (int i = 0; i < goal.parameters.data.size(); i++) {
//          msg.parameters.data.push_back(goal.parameters.data[i]);      \
//      }

      // No parameters are needed for isHolding
      //srv.request.parameters.data.push_back(-1);
      srv.request.isNew = 0;
      _srv_client.call(srv);

      if (_srv_client.call(srv))
      {
          //
          // ROS_INFO("State: %d", srv.response.estimated_state);  // Check if the condition is checked continuously
          ROS_INFO("Checking if it is reachable ");
      }
      else
      {
          ROS_ERROR("Failed to call service symbolic_perception");
      }

      if (srv.response.estimated_state == 0)
      {
         return BT::NodeStatus::SUCCESS;
      }
      else
         return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ConditionisAt::tick()
{

    // Service
      srv.request.state_index = 2;
      // Later you should define a proper port to connect to the symbolic_perception_server (state_index, parameters)
//      for (int i = 0; i < goal.parameters.data.size(); i++) {
//          msg.parameters.data.push_back(goal.parameters.data[i]);      \
//      }

      // No parameters are needed for isHolding
      //srv.request.parameters.data.push_back(-1);
      srv.request.isNew = 0;
      _srv_client.call(srv);

      if (_srv_client.call(srv))
      {
          //
          // ROS_INFO("State: %d", srv.response.estimated_state);  // Check if the condition is checked continuously
          ROS_INFO("Checking if it is at ");
      }
      else
      {
          ROS_ERROR("Failed to call service symbolic_perception");
      }

      if (srv.response.estimated_state == 0)
      {
         return BT::NodeStatus::SUCCESS;
      }
      else
         return BT::NodeStatus::FAILURE;
}

BT::NodeStatus btAIPClient::tick()
  {
      // Wait to see if the server is connected, up to to seconds. Otherwise set
      // FAILURE
      if(!_client.waitForServer(ros::Duration(2.0))){
        ROS_ERROR("Cannot contact AIPBT server");
        return BT::NodeStatus::FAILURE;
      }

      // The server is up, so we can get the input from the xml file
      defaultAIP_port goal;
      if ( !getInput<defaultAIP_port>("goal", goal))
      {
          throw BT::RuntimeError("Missing required input [goal]");
      }

      // Reset halt flag
      _halt_requested.store(false);

      ROS_INFO("Sending Goal prior=%i state_index=%i parameters[0] %f", goal.prior, goal.state_index, goal.parameters.data[0]);

      // Building goal message for the move_base
      behavior_control::AIPBTGoal msg;
      msg.prior = goal.prior;
      msg.state_index = goal.state_index;
      for (int i = 0; i < goal.parameters.data.size(); i++) {
          msg.parameters.data.push_back(goal.parameters.data[i]);      \
      }

      // Send te goal
      _client.sendGoal(msg);

      // We wait for the result but we also want to be able to abort if halt is
      // requested
      while (!_halt_requested && !_client.waitForResult(ros::Duration(0.02)))
      {
          /*
          if (goal.state_index == 0)
            ROS_INFO("Wanting isHolding");
          if (goal.state_index == 2)
            ROS_INFO("Wanting isAt");
          // Polling at 50Hz, do nothing
          */
      }

      // The BT halted this action
      /*
      if (_halt_requested){
        // _client.cancelAllGoals();
        ROS_ERROR("AIPBT halted");
        // Here I can add what to do if a part of the three is giving a halt
        return BT::NodeStatus::FAILURE;
      }
      */

      // If the server failed
      if (_client.getState()== actionlib::SimpleClientGoalState::ABORTED){
        ROS_ERROR("AIPBT failed");
        // Here I can add what to do if a part of the three is giving a halt
        return BT::NodeStatus::FAILURE;
      }

      // If all goes good
      if (_client.getState()== actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Success mate!");
        return BT::NodeStatus::SUCCESS;
      }
  }

}

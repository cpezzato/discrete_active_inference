#include "AIRLabAction_nodes.h"

// Action simple_action_client

// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time
BT_REGISTER_NODES(factory)
{
   AIRLabNodes::RegisterNodes(factory);
}

namespace AIRLabNodes
{

// Simple check for testing
BT::NodeStatus CheckBattery()
{
    std::cout << "[ Battery: OK ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CheckConditionFalse()
{
    std::cout << "[ Condition: False ]" << std::endl;
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ConditionisHolding::tick()
{
    // Service
      srv.request.state_index = 0;
      srv.request.isNew = 0;
      _srv_client.call(srv);

      if (_srv_client.call(srv)){
          ROS_INFO("Checking if it is holding");
          }
      else{
          ROS_ERROR("Failed to call service symbolic_perception");
      }

      if (srv.response.estimated_state == 0){
         return BT::NodeStatus::SUCCESS;
      }
      else
         return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ConditionisPlacedAt::tick()
{
    // Service
      srv.request.state_index = 4;
      srv.request.isNew = 0;
      _srv_client.call(srv);

      if (_srv_client.call(srv)){
          ROS_INFO("Checking if it is placed at");
          }
      else{
          ROS_ERROR("Failed to call service symbolic_perception");
      }

      if (srv.response.estimated_state == 0){
         return BT::NodeStatus::SUCCESS;
      }
      else
         return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveBaseAction::tick()
  {
      // Wait to see if the server is connected, up to to seconds. Otherwise set
      // FAILURE
      if(!_client.waitForServer(ros::Duration(2.0))){
        ROS_ERROR("Cannot contact move_base server");
        return BT::NodeStatus::FAILURE;
      }

      // The server is up, so we can get the input from the xml file
      Pose2D goal;
      if ( !getInput<Pose2D>("goal", goal))
      {
          throw BT::RuntimeError("missing required input [goal]");
      }

      // Reset halt flag
      _halt_requested.store(false);

      ROS_INFO("Sending Goal x=%f y=%f theta=%f", goal.x, goal.y, goal.theta);

      // Building goal message for the move_base
      move_base_msgs::MoveBaseGoal msg;
      msg.target_pose.header.frame_id = "map";
      msg.target_pose.header.stamp = ros::Time::now();

      msg.target_pose.pose.position.x = goal.x;
      msg.target_pose.pose.position.y = goal.y;
      tf::Quaternion rot = tf::createQuaternionFromYaw(goal.theta);
      tf::quaternionTFToMsg(rot, msg.target_pose.pose.orientation);

      // Send te goal
      _client.sendGoal(msg);

      // We wait for the result but we also want to be able to abort if halt is
      // requested
      while (!_halt_requested && !_client.waitForResult(ros::Duration(0.02)))
      {
          // Polling at 50Hz, do nothing
      }

      // The BT halted this action
      if (_halt_requested){
        _client.cancelAllGoals();
        ROS_ERROR("MoveBase halted");
        // Here I can add what to do if a part of the three is giving a halt
        return BT::NodeStatus::FAILURE;
      }

      // If the server failed
      if (_client.getState()!= actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_ERROR("MoveBase failed");
        // Here I can add what to do if a part of the three is giving a halt
        return BT::NodeStatus::FAILURE;
      }

      // If all goes good
      ROS_INFO("Target reached");
      return BT::NodeStatus::SUCCESS;
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

      ROS_INFO("Sending Goal prior=%i state_index=%i parameters[0] %f", goal.prior, goal.state_index, goal.parameters.data[0]);      // This is from te definition of Pose2D

      // Building goal message for the move_base
      discrete_ai::AIPBTGoal msg;
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
          if (goal.state_index == 0)
            ROS_INFO("Wanting isHolding");
          if (goal.state_index == 2)
            ROS_INFO("Wanting isAt");
          // Polling at 50Hz, do nothing
      }

      // The BT halted this action
      if (_halt_requested){
        // _client.cancelAllGoals();
        ROS_ERROR("AIPBT halted");
        // Here I can add what to do if a part of the three is giving a halt
        return BT::NodeStatus::FAILURE;
      }

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

BT::NodeStatus MyDummyAction::tick()
  {
      // The server is up, so we can get the input from the xml file
      Pose2D goal;

      ROS_INFO("Pretending to do stuff ACTION %f", goal.x);

      // Reset halt flag
      _halt_requested.store(false);

      SleepMS(5000);

      // If all goes good
      ROS_INFO("Target reached");
      return BT::NodeStatus::SUCCESS;
  }


}

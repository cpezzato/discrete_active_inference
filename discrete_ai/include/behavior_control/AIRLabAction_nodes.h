#pragma once

/*
    File: AIRLabAction_nodes.h

    Description: this file contains the definition of the templates for actions and conditions for the scripts
    using the BehaviorTree library from Davide Faconti.

    Author: Corrado Pezzato, TU Delft and AIRLab
    Date last update: 31.08.2020
*/
#ifndef AIRLABACTION_NODES_H
#define AIRLABACTION_NODES_H

#include "tf/transform_datatypes.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "std_msgs/Float64MultiArray.h"

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <behavior_control/AIPBTAction.h>
#include <behavior_control/symbolicPerception.h>

#include <actionlib/client/simple_action_client.h>

// Standard message for AIPBT_server
struct defaultAIP_port
{
    // The message contains tree entries: the first two are the prior preference over a state (prior), the second is the
    // state index to which the preference is related to:
    int prior, state_index;

    // The last entry is a Float64Multiarray of parameters, containing the parameters for a specific action. This can be
    // for instance the goal, the parametrisation for the action servers. See behavior/demo.xml for more info.
    std_msgs::Float64MultiArray parameters;
};

// Handy method to specify to wait for some time
inline void SleepMS(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

namespace BT
{
// Translating the string from xml to double numbers

template <> inline
defaultAIP_port convertFromString(StringView key)
{
    // Numbers separated by semicolons
    auto parts = BT::splitString(key, ';');
    if (parts.size() < 2)
    {
        throw BT::RuntimeError("Invalid input for default message, check documentation)");
    }
    else
    {
        defaultAIP_port output;
        // Save values from xml file to local structure
        output.prior = convertFromString<int>(parts[0]);
        output.state_index = convertFromString<int>(parts[1]);
        // Build the array for containing the parameters. The first two parameters
        for (int i = 2; i < parts.size(); i++) {
           output.parameters.data.push_back(convertFromString<double>(parts[i]));
        }
        return output;
    }
}

} // end namespace BT

// Definition of the AIRLab nodes
namespace AIRLabNodes
{


// Definition of the classes to check the conditions for active inference

// Condition isHolding
class ConditionisHolding : public BT::SyncActionNode
{
  public:
    ConditionisHolding(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {
        // This synchronous action connects to the symbolic_perception_server to retrieve a condition according to the
        // state index provided in the service request. The response from the server is either true or false
        _srv_client = n.serviceClient<behavior_control::symbolicPerception>("symbolic_perception");
        srv.request.parameters.data.push_back(-1);
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("message") };
    }

    private:
    ros::NodeHandle n;
    ros::ServiceClient _srv_client;
    behavior_control::symbolicPerception srv;
};

class ConditionisAt : public BT::SyncActionNode
{
  public:
    ConditionisAt (const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {
        // This synchronous action connects to the symbolic_perception_server to retrieve a condition according to the
        // state index provided in the service request. The response from the server is either true or false
        _srv_client = n.serviceClient<behavior_control::symbolicPerception>("symbolic_perception");
        srv.request.parameters.data.push_back(-1);
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("message") };
    }

    private:
    ros::NodeHandle n;
    ros::ServiceClient _srv_client;
    behavior_control::symbolicPerception srv;
};

class ConditionisReachable : public BT::SyncActionNode
{
  public:
    ConditionisReachable (const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {
        // This synchronous action connects to the symbolic_perception_server to retrieve a condition according to the
        // state index provided in the service request. The response from the server is either true or false
        _srv_client = n.serviceClient<behavior_control::symbolicPerception>("symbolic_perception");
        srv.request.parameters.data.push_back(-1);
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("message") };
    }

    private:
    ros::NodeHandle n;
    ros::ServiceClient _srv_client;
    behavior_control::symbolicPerception srv;
};

// While using active inference we only need one action type for the BT, since it is active inference which takes care
// of selecting the correct actions to execute. The action clients are then all declared in the AIPBT_server.
// This is an asynchronous operation that will run in a separate thread.

// Define the class which will contain te action client
class btAIPClient : public BT::AsyncActionNode
{
  public:
    btAIPClient(const std::string& name, const BT::NodeConfiguration& config)
      : BT::AsyncActionNode(name, config),
        _client("AIPBT_server", true) // "AIPBT_server" is the name of the server we want to build a client for
    {
    }
    // This asynchronous action requires the input port defaultAIP_port
    static BT::PortsList providedPorts()
    {
        return{BT::InputPort<defaultAIP_port>("goal") };
    }

    virtual BT::NodeStatus tick() override;

    // This overloaded method is used to stop the execution of this node. This is called by the BT automatically.
    // Whenever we provide an asynchronous action we need to provide a way to stop it
    virtual void halt() override
    {
        _halt_requested.store(true);
    }

  private:
    std::atomic_bool _halt_requested;
    typedef actionlib::SimpleActionClient<::behavior_control::AIPBTAction> myClient;      // <nameOfAction> This is the name of the action as in the automatically generated messages
    myClient _client;
};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<btAIPClient>("AIP_");             // These are the names to use in xml
    factory.registerNodeType<ConditionisHolding>("ConditionisHolding");
    factory.registerNodeType<ConditionisAt>("ConditionisAt");
    factory.registerNodeType<ConditionisReachable>("ConditionisReachable");
}

} // end namespace
#endif

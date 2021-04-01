#pragma once

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

// In here we defne the templates for the actions that we are going to include
// in the main file for behavior execution

// Custom type of data
struct Pose2D
{
    double x, y, theta;
};

// Standard message for AIPBT_server
struct defaultAIP_port
{
    int prior, state_index;
    std_msgs::Float64MultiArray parameters;
};

// Handy method to specify to wait
inline void SleepMS(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

namespace BT
{
// Translating the string from xml to double numbers
template <> inline
Pose2D convertFromString(StringView key)
{
    // three real numbers separated by semicolons
    auto parts = BT::splitString(key, ';');
    if (parts.size() != 3)
    {
        throw BT::RuntimeError("invalid input)");
    }
    else
    {
        Pose2D output;
        output.x     = convertFromString<double>(parts[0]);
        output.y     = convertFromString<double>(parts[1]);
        output.theta = convertFromString<double>(parts[2]);
        return output;
    }
}

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

namespace AIRLabNodes
{

// Simple check for testing
BT::NodeStatus CheckBattery();
BT::NodeStatus CheckConditionFalse();

// Example of custom SyncActionNode (synchronous action)
// with an input port.
class ConditionisHolding : public BT::SyncActionNode
{
  public:
    ConditionisHolding(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {
        _srv_client = n.serviceClient<behavior_control::symbolicPerception>("symbolic_perception");
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

class ConditionisPlacedAt : public BT::SyncActionNode
{
  public:
    ConditionisPlacedAt(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {
        _srv_client = n.serviceClient<behavior_control::symbolicPerception>("symbolic_perception");
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

// This is an asynchronous operation that will run in a separate thread.
// It requires the input port "goal".

class MoveBaseAction : public BT::AsyncActionNode
{
  public:
    MoveBaseAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::AsyncActionNode(name, config),
        _client("move_base", true)
    {
    }

    static BT::PortsList providedPorts()
    {
        return{BT::InputPort<Pose2D>("goal") };
    }

    virtual BT::NodeStatus tick() override;

    // This overloaded method is used to stop the execution of this node.
    // This is called by the BT automatically, you just need to implement this.
    // Whenever we provide an asynchronous action we need to provide a way to
    // stop it
    virtual void halt() override
    {
        _halt_requested.store(true);
    }

  private:
    std::atomic_bool _halt_requested;
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    MoveBaseClient _client;
};


// Define the class which will contain te action client
class btAIPClient : public BT::AsyncActionNode
{
  public:
    btAIPClient(const std::string& name, const BT::NodeConfiguration& config)
      : BT::AsyncActionNode(name, config),
        _client("AIPBT_server", true) // "AIPBT_server" is the name of the server we want to build a client for
    {
    }

    static BT::PortsList providedPorts()
    {
        return{BT::InputPort<defaultAIP_port>("goal") };
    }

    virtual BT::NodeStatus tick() override;

    // This overloaded method is used to stop the execution of this node.
    // This is called by the BT automatically, you just need to implement this.
    // Whenever we provide an asynchronous action we need to provide a way to
    // stop it
    virtual void halt() override
    {
        _halt_requested.store(true);
    }

  private:
    std::atomic_bool _halt_requested;
    typedef actionlib::SimpleActionClient<::behavior_control::AIPBTAction> myClient;      // <nameOfAction> This is the name of the action as in the automatically generated messages
    myClient _client;
};

class MyDummyAction : public BT::AsyncActionNode
{
  public:
    MyDummyAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::AsyncActionNode(name, config) // "AIPBT_server" is the name of the server we want to build a client for
    {
    }

    static BT::PortsList providedPorts()
    {
        return{BT::InputPort<defaultAIP_port>("goal") };
    }

    virtual BT::NodeStatus tick() override;

    // This overloaded method is used to stop the execution of this node.
    // This is called by the BT automatically, you just need to implement this.
    // Whenever we provide an asynchronous action we need to provide a way to
    // stop it
    virtual void halt() override
    {
        _halt_requested.store(true);
    }

    private:
    std::atomic_bool _halt_requested;
};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<MoveBaseAction>("MoveBase");
    factory.registerNodeType<btAIPClient>("AIP_");             // These are the names to use in xml
    factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));
    factory.registerSimpleCondition("CheckConditionFalse", std::bind(CheckConditionFalse));
    factory.registerNodeType<ConditionisHolding>("ConditionisHolding");
    factory.registerNodeType<ConditionisPlacedAt>("ConditionisPlacedAt");
    factory.registerNodeType<MyDummyAction>("MyDummyAction");

}

} // end namespace

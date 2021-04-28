#pragma once

#include "tf/transform_datatypes.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "std_msgs/Float64MultiArray.h"

#include <ros/ros.h>
#include <discrete_ai/AIPBTAction.h>
#include <discrete_ai/symbolicPerception.h>
#include <actionlib/client/simple_action_client.h>

// In here we defne the templates for the actions that we are going to include
// in the main file for behavior execution

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

}

namespace AIRLabNodes
{
// This is an asynchronous action representing the "prior" action node
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
    typedef actionlib::SimpleActionClient<::discrete_ai::AIPBTAction> myClient;      // <nameOfAction> This is the name of the action as in the automatically generated messages
    myClient _client;
};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<btAIPClient>("AIP_");             // These are the names to use in xml
}

} // end namespace

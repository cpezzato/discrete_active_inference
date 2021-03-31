/*
    File: demoBT.cpp

    Description: this file contains the definition of the BT for the execution of the AIRLab demo 2020.

    Author: Corrado Pezzato, TU Delft and AIRLab
    Date last update: 31.08.2020
*/

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behavior_control/AIRLabAction_nodes.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"

using namespace BT;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_bt");
    std::string fileName;
    if (argc == 2) {
	    fileName = argv[1];
    }
    else{
	    ROS_WARN("You did not specify a behavior xml file, demo_pickPlace is used");
	    fileName = "demo_pickPlace.xml";
    }
    std::string fullFileName = "/home/pandacontrol/demo_ws/src/behavior_control/src/behaviors/" + fileName;
    ros::NodeHandle nh("~");
    BT::BehaviorTreeFactory factory;

    // Register the needed actions into the factory
    using namespace AIRLabNodes;
    // Register here the active inference nodes. They are all of the same type <btAIPclient> action.
    // The only thing that changes are the parameters passed through the port, in which you indicate
    // the state index, the desired prior, and the parameters to execute an action. These are the names to use in xml
    factory.registerNodeType<btAIPClient>("AIP_isAt");
    factory.registerNodeType<btAIPClient>("AIP_isHolding");
    factory.registerNodeType<btAIPClient>("AIP_isReachable");
    factory.registerNodeType<btAIPClient>("AIP_isSpotEmpty");        // These are the names to use in xml
    factory.registerNodeType<btAIPClient>("AIP_isPlacedAt");        // These are the names to use in xml

    // Register conditions
    factory.registerNodeType<ConditionisHolding>("ConditionisHolding");
    factory.registerNodeType<ConditionisAt>("ConditionisAt");
    factory.registerNodeType<ConditionisReachable>("ConditionisReachable");


    // Define the behavior tree from xml format
    // auto tree = factory.createTreeFromFile("/home/pandacontrol/demo_ws/src/behavior_control/src/behaviors/demo_pickPlace.xml");
    auto tree = factory.createTreeFromFile(fullFileName);
    // Also possible to create it from text, which should be added before the main in this file
   //auto tree = factory.createTreeFromText(mpc_demo);

    // Log to keep track of the behavior tree status on the terminal
    BT::StdCoutLogger logger_cout(tree);
    BT::NodeStatus status = NodeStatus::RUNNING;

    // Keep executing the tick until the root returns either SUCCESS or FAILURE
    while(ros::ok() && (status == NodeStatus::RUNNING))
    {
        status = tree.tickRoot();
        // ROS_INFO("Ticking root at 1Hz");
        // This specify at which frequency the BT is ticked
        SleepMS(1000);
    }
    return 0;
}

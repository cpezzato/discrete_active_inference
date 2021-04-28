#include "behaviortree_cpp_v3/bt_factory.h"
#include "AIRLabAction_nodes.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"

using namespace BT;

static const char *tiago_stock_shelf = R"(
<root main_tree_to_execute = "MainTree" >
    <BehaviorTree ID="MainTree">
            <ReactiveSequence name="root">
                    <AIP_isHolding      goal="0; 0; 1.3; 0.3; 0; 0; 0; 0; 1; 1;"/> <!-- State value; state index; x_obj_loc; y_obj_loc; object index -->
                    <AIP_isAt           goal="0; 2; 0.3; -1.1; 0; 0; 0; -0.707; 0.707"/> <!-- State value; state index; x, y, z, quaternion xyzw -->
                    <AIP_isPlacedAt     goal="0; 4; 0.3; -1.1; 0; 0; 0; -0.707; 0.707; 1;"/> <!-- State value; state index; x_obj_loc; y_obj_loc; object index -->
            </ReactiveSequence>
    </BehaviorTree>
</root>

 )";

//  USAGE AIP_*: prior (0 or 1); state_index; parameters[*]
//  States indexes: isHolding, isReachable, isAt, isPlacedAt (indexes 0, 1, 2, 4)

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_bt");
    ros::NodeHandle nh("~");

    BT::BehaviorTreeFactory factory;
    // Register the needed actions into the factory
    using namespace AIRLabNodes;
    // Register here the active inference nodes. They are all of the same type <btAIPclient> action.
    // The only thing that changes are the parameters passed through the port, in which you indicate
    // the state index, the desired prior, and the parameters to execute an action. These are according to documentation
    factory.registerNodeType<btAIPClient>("AIP_isAt");             // These are the names to use in xml
    factory.registerNodeType<btAIPClient>("AIP_isHolding");        // These are the names to use in xml
    //factory.registerNodeType<btAIPClient>("AIP_isSpotEmpty");        // These are the names to use in xml
    factory.registerNodeType<btAIPClient>("AIP_isPlacedAt");        // These are the names to use in xml

    // Define the behavior tree from xml format
    auto tree = factory.createTreeFromText(tiago_stock_shelf);

    // You can also get your behavior tree from xml file
    //auto tree = factory.createTreeFromFile("/ABSOLUTE_PATH_TO/stock_shelf.xml");
    
    // Log to keep track of the behavior tree status on the terminal
    BT::StdCoutLogger logger_cout(tree);
    BT::NodeStatus status = NodeStatus::RUNNING;

    //---------------------------------------
    // keep executing tick until it returns either SUCCESS or FAILURE
    while(ros::ok() && (status == NodeStatus::RUNNING))
    {
        status = tree.tickRoot();
        // ROS_INFO("Ticking root at 1Hz");
        // This specify at which frequency the BT is ticked
        SleepMS(1000);
    }
    return 0;
}

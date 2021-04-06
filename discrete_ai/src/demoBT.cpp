#include "behaviortree_cpp_v3/bt_factory.h"
#include "AIRLabAction_nodes.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"

using namespace BT;

static const char* tiago_test = R"(
<root main_tree_to_execute = "MainTree" >
    <BehaviorTree ID="MainTree">
            <Sequence name="root">
                    <AIP_isAt    goal="0; 2; 0.31; -0.57"/> 
            </Sequence>
    </BehaviorTree>
</root>
 )";

static const char* tiago_pick_place_real = R"(
<root main_tree_to_execute = "MainTree" >
    <BehaviorTree ID="MainTree">
            <ReactiveSequence name="root">
                    <AIP_isHolding    goal="0; 0; 0.56; -0.4; 1;"/> <!-- State value; state index; x_obj_loc; y_obj_loc; object index -->
                    <AIP_isAt    goal="0; 2; 2.6; 0.93"/> <!-- Location at x = 2.4, y = 1 -->
                    <AIP_isPlacedAt    goal="0; 4; 2.6; 0.93; 1;"/> <!-- State value; state index; x_obj_loc; y_obj_loc; object index -->
            </ReactiveSequence>
    </BehaviorTree>
</root>

 )";

 static const char* tiago_pick_place_sim = R"(
<root main_tree_to_execute = "MainTree" >
    <BehaviorTree ID="MainTree">
            <ReactiveSequence name="root">
                    <AIP_isHolding    goal="0; 0; 1.3; 0; 1;"/> <!-- State value; state index; x_obj_loc; y_obj_loc; object index -->
                    <!-- <AIP_isAt    goal="0; 2; 1.3; 0"/> <!-- Location at x = 1.3, y = 0>
                    <AIP_isHolding    goal="0; 0; -1.3; 0; 1;"/> <!-- State value; state index; x_obj_loc; y_obj_loc; object index -->
            </ReactiveSequence>
    </BehaviorTree>
</root>

 )";

//  USAGE AIP_*: prior (0 or 1); state_index; parameters[*]
//  States order: isHolding, isReachable, isAt (indexes 0, 1, 2)

//  Parameters:
//  isHolding: (x; y; z; obj_id) object id, and position of the object
//  isReachable: (x; y) location of the base
//  isAt: (x; y) location of the base

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_bt");
    ros::NodeHandle nh("~");

    BT::BehaviorTreeFactory factory;
    // Register the needed actions into the factory
    using namespace AIRLabNodes;
    factory.registerNodeType<MoveBaseAction>("MoveBase");
    // Register here the active inference nodes. They are all of the same type <btAIPclient> action.
    // The only thing that changes are the parameters passed through the port, in which you indicate
    // the state index, the desired prior, and the parameters to execute an action. These are according to documentation
    factory.registerNodeType<btAIPClient>("AIP_isAt");             // These are the names to use in xml
    factory.registerNodeType<btAIPClient>("AIP_isHolding");        // These are the names to use in xml
    factory.registerNodeType<btAIPClient>("AIP_isSpotEmpty");        // These are the names to use in xml
    factory.registerNodeType<btAIPClient>("AIP_isPlacedAt");        // These are the names to use in xml

    factory.registerSimpleCondition("BatteryOK", std::bind(CheckBattery));
    factory.registerNodeType<ConditionisHolding>("ConditionisHolding");
    factory.registerNodeType<ConditionisPlacedAt>("ConditionisPlacedAt");
    factory.registerSimpleCondition("FalseCondition", std::bind(CheckConditionFalse));
    factory.registerNodeType<MyDummyAction>("MyDummyAction");


    // Define the behavior tree from xml format
    auto tree = factory.createTreeFromText(tiago_pick_place_real);
    //auto tree = factory.createTreeFromText(tiago_pick_place_sim);

    //auto tree = factory.createTreeFromFile("/home/corrado/simulations/my_ws/src/behavior_control/src/behaviors/xml_conflicts.xml");
    
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

#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/bt_service_node.hpp>
#include "dmke_interface/action/set_position.hpp"
#include "dmke_interface/srv/get_position.hpp"
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
#include <unordered_map>
#include "dmke_bt_test/set_position_action.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>


// #include <behaviortree_ros2/plugins.hpp>


using GetPosition = dmke_interface::srv::GetPosition;
using namespace BT;
// let's define these, for brevity
// using SetPosition = dmke_interface::action::SetPosition;
// using GoalHandleSetPosition = rclcpp_action::ServerGoalHandle<SetPosition>;

// PUT ALL THE CLIENT IN THIS FILE SO THAT BT CAN TICK WHEN CALLED.
// - Service Client (dmke)
//  - Action client (dmke)


// ACTION //
class DMKESetPosition: public RosActionNode<dmke_interface::action::SetPosition>
{
public:
  DMKESetPosition(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosActionNode<dmke_interface::action::SetPosition>(name, conf, params)
  {  }

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosActionNode::providedBasicPorts()
  // static PortsList providedPorts()
  // {
  //   return providedBasicPorts({InputPort<unsigned>("target_position")});
  // }
  
  static PortsList providedPorts() {
        return providedBasicPorts({InputPort<std::string>("action_name"), 
                                  InputPort<int>("target_position")});
    }



  bool setGoal(RosActionNode::Goal& goal) override {
    // Retrieve target_position from the input port
    if (!getInput("target_position", goal.target_position)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get target_position from input port");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Using value for 'target_position': %u", goal.target_position);

    // Return true, indicating successful goal setting
    return true;
  }
  // This is called when the TreeNode is ticked and it should
  // send the request to the action server
  // bool setGoal(RosActionNode::Goal& goal) override {
  //   // get "order" from the Input port
  //   // getInput("order", goal.order);

  //   // Create an instance of SendGoal
  //   // SendGoal sendGoalObj;

  //   // getInput("target_position", goal.target_position);
  //   // Use goal.order directly, since it is set in the constructor
  //   RCLCPP_INFO(node_->get_logger(), "Using value for 'target_position': %u", goal.target_position);
  
   // Return true, indicating successful goal setting
    // return true;

    // Create a goal message
    // auto goal_msg = std::make_shared<example_interfaces::action::SetPosition::Goal>();
    // goal_msg->target_position = target_position;

    // if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    //     RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    //     return;
    // }

    // Return true, assuming the goal was set correctly
//     return true;
// }
  
  // Callback executed when the reply is received.
  // Based on the reply you may decide to return SUCCESS or FAILURE.
  NodeStatus onResultReceived(const WrappedResult& wr) override
  {
    std::stringstream ss;
    ss << "Result received: ";
    auto result = wr.result->success;
    ss << result << " ";
  
    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    return NodeStatus::SUCCESS;
    // try {
    //     auto result = future.get().get();
    //     RCLCPP_INFO(rclcpp::get_logger("get_result_callback"), "Result received: %u", result.success);
    // } catch (const std::exception& e) {
    //     RCLCPP_ERROR(rclcpp::get_logger("get_result_callback"), "Exception while calling service: %s", e.what());
    // }
  }

  // Callback invoked when there was an error at the level
  // of the communication between client and server.
  // This will set the status of the TreeNode to either SUCCESS or FAILURE,
  // based on the return value.
  // If not overridden, it will return FAILURE by default.
  virtual NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
    return NodeStatus::FAILURE;
  }

  // we also support a callback for the feedback, as in
  // the original tutorial.
  // Usually, this callback should return RUNNING, but you
  // might decide, based on the value of the feedback, to abort
  // the action, and consider the TreeNode completed.
  // In that case, return SUCCESS or FAILURE.
  // The Cancel request will be send automatically to the server.
  NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Current Position: ";
    auto current_pos = feedback->current_position;
    ss << current_pos;
    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    return NodeStatus::RUNNING;
  }
};


// SERVICE //

class GetPositionService: public RosServiceNode<GetPosition>
{
  public:
    GetPositionService(const std::string& name,
                    const NodeConfig& conf,
                    const RosNodeParams& params)
      : RosServiceNode<GetPosition>(name, conf, params)
    {}

    // The specific ports of this Derived class
    // should be merged with the ports of the base class,
    // using RosServiceNode::providedBasicPorts()
    static PortsList providedPorts()
    {
      return providedBasicPorts({
        InputPort<std::string>("service_name"),
        InputPort<int>("SupposedPosition"),
        // OutputPort<int>("position")
      });
    }

    // This is called when the TreeNode is ticked and it should
    // send the request to the service provider
    bool setRequest(Request::SharedPtr& request) override
    {
      // must return true if we are ready to send the request
      (void)request;
      return true;
    }

    // Callback invoked when the answer is received.
    // It must return SUCCESS or FAILURE
    NodeStatus onResponseReceived(const Response::SharedPtr& response) override
  {
    // Log
    auto positiongot = response->position;
    
    if (positiongot <= getInput<int>("SupposedPosition").value() + 20 ||
         positiongot >= getInput<int>("SupposedPosition").value() - 20) {
         std::stringstream ss;
         ss << this->name() << " -> Position Obtained: " << positiongot;
         RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

         // setOutput("position", response->position);
         return NodeStatus::SUCCESS;
         }


    else{
      std::stringstream ss;
      ss << this->name() << " -> Position NOT within bounds: " << response->position;
      RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

      // setOutput("position", response->position);
      return NodeStatus::FAILURE;
    }
  }


    // Callback invoked when there was an error at the level
    // of the communication between client and server.
    // This will set the status of the TreeNode to either SUCCESS or FAILURE,
    // based on the return value.
    // If not overridden, it will return FAILURE by default.
    virtual NodeStatus onFailure(ServiceNodeErrorCode error) override
    {
      std::stringstream ss;
      ss << this->name() << " -> Error: " << error;
      RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

      return NodeStatus::FAILURE;
    }
};

//<?xml version="1.0" encoding="UTF-8"?>
// Simple tree, used to execute once each action.
static const char* xml_text = R"(
<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4">
  <BehaviorTree ID="ROSwithBT_test1">
    <Repeat num_cycles="2">
      <Sequence>
        <Delay delay_msec="3000">
          <Fallback>
            <GetPosition service_name="/get_position" SupposedPosition="500000"/>
            <DMKESetPosition action_name="/set_position" target_position="500000"/>
          </Fallback>
        </Delay>
        <Delay delay_msec="3000">
          <Fallback>
            <GetPosition service_name="/get_position" SupposedPosition="0"/>
            <DMKESetPosition action_name="/set_position" target_position="0"/>
          </Fallback>
        </Delay>
      </Sequence>
    </Repeat>
  </BehaviorTree>

  <!-- Description of Node Models (used by Groot) -->
  <TreeNodesModel/>

</root>
)";

// <root BTCPP_format="4">
//   <BehaviorTree ID="ROSwithBT_test1">
//     <Repeat num_cycles="2">
//       <Sequence>
//         <Delay delay_msec="3000">
//           <DMKESetPosition action_name="/set_position" target_position="500000"/>
//         </Delay>
//         <Delay delay_msec="3000">
//           <DMKESetPosition action_name="/set_position" target_position="0"/>
//         </Delay>
//       </Sequence>
//     </Repeat>
//   </BehaviorTree>
//   <TreeNodesModel/>
// </root>
// )";

// <?xml version="1.0" encoding="UTF-8"?>
// <root BTCPP_format="4">
//   <BehaviorTree ID="ROSwithBT_test1">
//     <Repeat num_cycles="3">
//       <Sequence>
//         <Delay delay_msec="3000">
//           <Fallback>
//             <GetPosition service_name="/get_position" SupposedPosition="0"/>
//             <DMKESetPosition action_name="/set_position" target_position="500000"/>
//           </Fallback>
//         </Delay>
//         <Delay delay_msec="3000">
//           <Fallback>
//             <GetPosition service_name="/get_position" SupposedPosition="500000"/>
//             <DMKESetPosition action_name="/set_position" target_position="0"/>
//           </Fallback>
//         </Delay>
//       </Sequence>
//     </Repeat>
//   </BehaviorTree>

//   <!-- Description of Node Models (used by Groot) -->
//   <TreeNodesModel/>

// </root>




int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  BehaviorTreeFactory factory;

  auto bt_node = std::make_shared<rclcpp::Node>("dmke_bt_test2");
  // provide the ROS node and the name of the action service
  RosNodeParams setpos_params; 
  setpos_params.nh = bt_node;
  // setpos_params.default_port_value = "set_position_server";

  factory.registerNodeType<DMKESetPosition>("DMKESetPosition", setpos_params);
  factory.registerNodeType<GetPositionService>("GetPosition", setpos_params);
  // factory.registerNodeType<DMKEGetPosService>("GetPosition", params);

  // Create the behavior tree using the XML description
  auto tree = factory.createTreeFromText(xml_text);

  // Run the behavior tree until it finishes
  tree.tickWhileRunning();
  // Spin
  rclcpp::spin(bt_node);
  rclcpp::shutdown();
  return 0;

  // Create node
  // rclcpp::init(argc, argv);
  // auto bt_server_node = std::make_shared<ArmBtServer>("arm_bt_server");

  // // Create BT factory
  // BehaviorTreeFactory factory;
  // RosNodeParams bt_server_params; 
  // bt_server_params.nh = bt_server_node;

  // // Registor tree nodes
  // factory.registerNodeType<GetPositionService>("GetPositionService", bt_server_params);
  // factory.registerNodeType<SetPositionAction>("SetPositionAction", bt_server_params);
  // factory.registerNodeType<InverseKinematicsService>("InverseKinematicsService", bt_server_params);

  // // Create the behavior tree using the XML description
  // Tree home_tree = factory.createTreeFromText(home_xml);
  // Tree target_tree = factory.createTreeFromText(target_xml);
  // bt_server_node->set_tree(ArmBtServer::BtType::HOME, &home_tree);
  // bt_server_node->set_tree(ArmBtServer::BtType::TARGET, &target_tree);

  // // Spin
  // rclcpp::spin(bt_server_node);
  // rclcpp::shutdown();
  // return 0;

}

#include <behaviortree_ros2/bt_action_node.hpp>
#include <iostream>

using namespace BT;
// let's define these, for brevity
using SetPosition = dmke_interfaces::action::SetPosition;
using GoalHandleSetPosition = rclcpp_action::ServerGoalHandle<SetPosition>;

// PUT ALL THE CLIENT IN THIS FILE SO THAT BT CAN TICK WHEN CALLED.
// - Service Client (dmke)
//  - Action client (dmke)



// ACTION //
class DMKESetPosition: public RosActionNode<SetPosition>
{
public:
  DMKESetPosition(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosActionNode<SetPosition>(name, conf, params)
  {
    // Set default value for order
    goal_.target_position = 500000; // Default value of 10, for example
  }

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosActionNode::providedBasicPorts()
  // static PortsList providedPorts()
  // {
  //   return providedBasicPorts({InputPort<unsigned>("target_position")});
  // }

  static PortsList providedPorts() {
        return providedBasicPorts({ InputPort<std::string>("action_name"), InputPort<int>("target_position") });
    }


  // This is called when the TreeNode is ticked and it should
  // send the request to the action server
  bool setGoal(RosActionNode::Goal& goal) override {
    // get "order" from the Input port
    // getInput("order", goal.order);

    // Create an instance of SendGoal
    // SendGoal sendGoalObj;

    // getInput("target_position", goal.target_position);
    // Use goal.order directly, since it is set in the constructor
    RCLCPP_INFO(node_->get_logger(), "Using value for 'target_position': %u", goal.target_position);
  
   // Return true, indicating successful goal setting
    // return true;

    // Create a goal message
    // auto goal_msg = std::make_shared<example_interfaces::action::SetPosition::Goal>();
    // goal_msg->target_position = target_position;

    if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        return;
    }

    // Return true, assuming the goal was set correctly
    return true;
}
  
  // Callback executed when the reply is received.
  // Based on the reply you may decide to return SUCCESS or FAILURE.
  NodeStatus onResultReceived(const WrappedResult& wr) override
  {
    // std::stringstream ss;
    // ss << "Result received: ";
    // for (auto number : wr.result->sequence) {
    //   ss << number << " ";  
    // }
    // RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    // return NodeStatus::SUCCESS;
    try {
        auto result = future.get().get();
        RCLCPP_INFO(rclcpp::get_logger("get_result_callback"), "Result received: %u", result.success);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("get_result_callback"), "Exception while calling service: %s", e.what());
    }
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
    ss << "Next number in sequence received: ";
    // for (auto number : feedback->partial_sequence) {
    //   ss << number << " ";
    // }
    auto feedback = feedback_msg->feedback;
    RCLCPP_INFO(this->get_logger(), "Received feedback: Current position is %d", feedback.current_position);
    return NodeStatus::RUNNING;
  }
};


// SERVICE //
// class DMKEGetPosition: public RosServiceNode<dmke_interfaces::srv::GetPosition>
// {
//   public:
//     GetPositionService(const std::string& name,
//                     const NodeConfig& conf,
//                     const RosNodeParams& params)
//       : RosServiceNode<dmke_interface::srv::GetPosition>(name, conf, params)
//     {}

//     // The specific ports of this Derived class
//     // should be merged with the ports of the base class,
//     // using RosServiceNode::providedBasicPorts()
//     static PortsList providedPorts()
//     {
//       return providedBasicPorts({
//         // InputPort<std::string>("service_name"),
//         OutputPort<int>("position_gotten")
//       });
//     }

//     // This is called when the TreeNode is ticked and it should
//     // send the request to the service provider
//     bool setRequest(Request::SharedPtr& request) override
//     {
//       // must return true if we are ready to send the request
//       (void)request;
//       return true;
//     }

//     // Callback invoked when the answer is received.
//     // It must return SUCCESS or FAILURE
//     NodeStatus onResponseReceived(const Response::SharedPtr& response) override
//     {
//       // Log
//       std::stringstream ss;
//       ss << this->name() << " -> Current Position: " << response->position;
//       RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

//       setOutput("position_gotten", response->position);
//       return NodeStatus::SUCCESS;
//     }

//     // Callback invoked when there was an error at the level
//     // of the communication between client and server.
//     // This will set the status of the TreeNode to either SUCCESS or FAILURE,
//     // based on the return value.
//     // If not overridden, it will return FAILURE by default.
//     virtual NodeStatus onFailure(ServiceNodeErrorCode error) override
//     {
//       std::stringstream ss;
//       ss << this->name() << " -> Error: " << error;
//       RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

//       return NodeStatus::FAILURE;
//     }
// };




// class DMKEGetPosService: public RosServiceNode<GetPosition>
// {
// public:

//   AddTwoIntsNode(const std::string& name,
//                   const NodeConfig& conf,
//                   const RosNodeParams& params)
//     : RosServiceNode<AddTwoInts>(name, conf, params)
//   {}

//   static PortsList providedPorts()
//   {
//     // Since no input ports are needed, return an empty list
//     return {};
//   }

//   bool setRequest(Request::SharedPtr& request) override
//   {
//     // This function is not needed since no request is required
//     // Return false to indicate no request is sent
//     return false;
//   }

//   NodeStatus onResponseReceived(const Response::SharedPtr& response) override
//   {
//     RCLCPP_INFO(node_->get_logger(), "Current Position: %ld", response->position_gotten);
//     return NodeStatus::SUCCESS;
//   }

//   virtual NodeStatus onFailure(ServiceNodeErrorCode error) override
//   {
//     RCLCPP_ERROR(node_->get_logger(), "Error: %u", error);
//     return NodeStatus::FAILURE;
//   }
// };



//<?xml version="1.0" encoding="UTF-8"?>
// Simple tree, used to execute once each action.
static const char* xml_text = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="ROSwithBT_test1">
    <Repeat num_cycles="3">
      <Sequence>
        <Delay delay_msec="3000">
          <DMKESetPosition action_name="/OpenUC" target_position="500000"/>
        </Delay>
        <Delay delay_msec="3000">
          <DMKESetPosition action_name="/CloseUC" target_position="0"/>
        </Delay>
      </Sequence>
    </Repeat>
  </BehaviorTree>
  <TreeNodesModel/>
</root>
)";



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  BehaviorTreeFactory factory;

  auto node = std::make_shared<rclcpp::Node>("setposition_action_client");
  // provide the ROS node and the name of the action service
  RosNodeParams params; 
  params.nh = node;
  params.default_port_value = "set_position_server";

  factory.registerNodeType<DMKESetPosition>("SetPosition", params);
  // factory.registerNodeType<DMKEGetPosService>("GetPosition", params);

  // Create the behavior tree using the XML description
  auto tree = factory.createTreeFromText(xml_text);

  // Run the behavior tree until it finishes
  tree.tickWhileRunning();

  return 0;

}

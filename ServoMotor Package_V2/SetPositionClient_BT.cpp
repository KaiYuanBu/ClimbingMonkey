#include <behaviortree_ros2/bt_action_node.hpp>
#include  

using namespace BT;

class DMKESetPosition: public RosActionNode<SetPosition>
{
public:
  DMKESetPosition(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosActionNode<SetPosition>(name, conf, params)
  {}

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosActionNode::providedBasicPorts()
  static PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<unsigned>("target_position")});
  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the action server
  bool setGoal(RosActionNode::Goal& goal) override {
    // get "order" from the Input port
    // getInput("order", goal.order);

    // Create an instance of SendGoal
    // SendGoal sendGoalObj;

    getInput("target_position", goal.target_position);

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
        RCLCPP_INFO(rclcpp::get_logger("get_result_callback"), "Result received: %d", result.success);
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
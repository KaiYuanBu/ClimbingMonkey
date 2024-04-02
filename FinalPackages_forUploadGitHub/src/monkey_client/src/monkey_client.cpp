#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include "monkey_interface/action/set_position.hpp"
#include "monkey_interface/action/set_extension.hpp"
#include "monkey_interface/srv/get_position.hpp"
#include "monkey_interface/srv/get_extension.hpp"
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
#include <unordered_map>
#include <sensor_msgs/msg/image.hpp>
// #include "dmke_bt_test/set_position_action.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <chrono>
#include <math.h>
#include <fstream>
#include <stdexcept>

// #include <behaviortree_ros2/plugins.hpp>


// using GetPosition = monkey_interface::srv::GetPosition;
using namespace BT;
using namespace std::chrono;
// let's define these, for brevity
// using SetPosition = monkey_interface::action::SetPosition;
// using GoalHandleSetPosition = rclcpp_action::ServerGoalHandle<SetPosition>;

// PUT ALL THE CLIENT IN THIS FILE SO THAT BT CAN TICK WHEN CALLED.
// - Service Client (dmke)
//  - Action client (dmke)
int count = 0;
float new_val = 0;

int x = 0; //Climb Up steps number
int n = 0; //Number of cycles to climb
int y = 0; //Climb Down steps number


void save_value_to_file(int number, const std::string& file_path) {
    std::ofstream file(file_path);
    if (file.is_open()) {
        file << number;
        file.close();
    } else {
        throw std::runtime_error("Unable to open file for writing.");
    }
}

int read_value_from_file(const std::string& file_path) {
    int value = 0;
    std::ifstream file(file_path);
    if (file.is_open()) {
        file >> value;
        file.close();
    } else {
        // If file not found, create the file with default value
        std::ofstream new_file(file_path);
        if (new_file.is_open()) {
            new_file << value;
            new_file.close();
        } else {
            throw std::runtime_error("Unable to open file for writing.");
        }
    }
    return value;
}

// ##### ========== ---------- DMKE ----------  ========== #### //
// ACTION //
class DMKESetPosition: public RosActionNode<monkey_interface::action::SetPosition>
{
public:
  DMKESetPosition(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosActionNode<monkey_interface::action::SetPosition>(name, conf, params)
  {  }

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosActionNode::providedBasicPorts()
  // static PortsList providedPorts()
  // {
  //   return providedBasicPorts({InputPort<unsigned>("target_position")});
  // }
  
  static PortsList providedPorts() {
        return providedBasicPorts({InputPort<int>("node_id"),
                                  InputPort<std::string>("action_name"), 
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
  
  // Callback executed when the reply is received.
  // Based on the reply you may decide to return SUCCESS or FAILURE.
  NodeStatus onResultReceived(const WrappedResult& wr) override
  {
    std::stringstream ss;
    ss << "Result received: ";
    auto result = wr.result->success_pos;
    ss << result << " ";

    if (result == 1){
      RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
      // int prev_n = read_value_from_file("NumberofCycles.txt");
      // int currnetx = read_value_from_file("ClimbUp");
      // if (prev_n != currnetx){
      //   x++;
      //   save_value_to_file(x, "ClimbUp");
      // }

      // if (prev_n == currnetx){
      //   x++;
      //   save_value_to_file(x, "ClimbUp");
      // }
      return NodeStatus::SUCCESS;
    }

    else{
      RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
      return NodeStatus::FAILURE;
    }
    
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

class DMKEGetPosition: public RosServiceNode<monkey_interface::srv::GetPosition>
{
  public:
    DMKEGetPosition(const std::string& name,
                    const NodeConfig& conf,
                    const RosNodeParams& params)
      : RosServiceNode<monkey_interface::srv::GetPosition>(name, conf, params)
    {}

    // The specific ports of this Derived class
    // should be merged with the ports of the base class,
    // using RosServiceNode::providedBasicPorts()
    static PortsList providedPorts()
    {
      return providedBasicPorts({
        InputPort<int>("node_id"),
        InputPort<std::string>("service_name"),
        InputPort<int>("target_pos"),
        // OutputPort<int>("position")
      });
    }

    // This is called when the TreeNode is ticked and it should
    // send the request to the service provider
    bool setRequest(Request::SharedPtr& request) override
    {
      // must return true if we are ready to send the request
      // (void)request;
      getInput("node_id", request->node_id);
      return true;
    }

    // Callback invoked when the answer is received.
    // It must return SUCCESS or FAILURE

    
    // DO PRECONDITION HERE INSTEAD OF COMPARING VALUES INSIDE, IT SEEMS THAT IT JUST RETURNS THE FIRST STATUS FOR SOME REASON
    NodeStatus onResponseReceived(const Response::SharedPtr& response) override
  {
    // Log
    int positiongot = response->positiongot;
    
    if (positiongot > getInput<int>("target_pos").value() + 5000 || positiongot < getInput<int>("target_pos").value() - 5000) 
         {
         std::stringstream ss;
         ss << this->name() << " -> Position NOT within bounds (NODESTATUS:FAILURE): " << positiongot;
         RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

         // setOutput("position", response->position);
         return NodeStatus::FAILURE;
         }


    else{
      std::stringstream ss;
      ss << this->name() << " -> Position within bounds (NODESTATUS:SUCCESS): " << positiongot;
      RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

      // setOutput("position", response->position);
      return NodeStatus::SUCCESS;
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



// ##### ========== ---------- CYLINDER ----------  ========== #### //
// ACTION //
class CylinderSetExtension: public RosActionNode<monkey_interface::action::SetExtension>
{
public:
  CylinderSetExtension(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosActionNode<monkey_interface::action::SetExtension>(name, conf, params)
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
                                  InputPort<float>("target_extension")});
    }

  // This is called when the TreeNode is ticked and it should
  // send the request to the action server
  bool setGoal(RosActionNode::Goal& goal) override {
    // Retrieve target_position from the input port
    if (!getInput("target_extension", goal.target_extension)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get target_extension from input port");
        return false;
    }

    //  if (!action_client_->wait_for_action_server(std::chrono::seconds(20))) {
    //     RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    //     return;
    //  }

    RCLCPP_INFO(node_->get_logger(), "Using value for 'target_extension': %f", goal.target_extension);

    // Return true, indicating successful goal setting
    return true;
  }
  
  // Callback executed when the reply is received.
  // Based on the reply you may decide to return SUCCESS or FAILURE.
  NodeStatus onResultReceived(const WrappedResult& wr) override
  {
    std::stringstream ss;
    ss << "Result received: ";
    auto result = wr.result->success_ext;
    ss << result << " ";
  
    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    return NodeStatus::SUCCESS;
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
    ss << "Current Extension: ";
    auto current_ext = feedback->current_extension;
    ss << current_ext;
    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    return NodeStatus::RUNNING;
  }
};



// SERVICE //

class CylinderGetExtension: public RosServiceNode<monkey_interface::srv::GetExtension>
{
  public:
    CylinderGetExtension(const std::string& name,
                    const NodeConfig& conf,
                    const RosNodeParams& params)
      : RosServiceNode<monkey_interface::srv::GetExtension>(name, conf, params)
    {}

    // The specific ports of this Derived class
    // should be merged with the ports of the base class,
    // using RosServiceNode::providedBasicPorts()
    static PortsList providedPorts()
    {
      return providedBasicPorts({
        InputPort<std::string>("service_name"),
        InputPort<float>("target_ext"),
        // OutputPort<int>("position")
      });
    }

    // This is called when the TreeNode is ticked and it should
    // send the request to the service provider
    bool setRequest(Request::SharedPtr& request) override
    {
      // must return true if we are ready to send the request
      (void)request;
      // getInput("node_id", request->node_id);
      return true;
    }

    // Callback invoked when the answer is received.
    // It must return SUCCESS or FAILURE

    
    NodeStatus onResponseReceived(const Response::SharedPtr& response) override
  {
    // Log
    float extensiongot = response->extensiongot;
    
    if (extensiongot > getInput<float>("target_ext").value() + 0.05 || extensiongot < getInput<float>("target_ext").value() - 0.05) 
         {
         std::stringstream ss;
         ss << this->name() << " -> Position NOT within bounds (NODESTATUS:FAILURE): " << extensiongot;
         RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

         // setOutput("position", response->position);
         return NodeStatus::FAILURE;
         }


    else{
      std::stringstream ss;
      ss << this->name() << " -> Position within bounds (NODESTATUS:SUCCESS): " << extensiongot;
      RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

      // setOutput("position", response->position);
      return NodeStatus::SUCCESS;
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





// ZED Camera Topic Sub //
class HeightDetection : public RosTopicSubNode<sensor_msgs::msg::Image>
{
public:
  HeightDetection(const std::string &name,
                  const NodeConfig &conf,
                  const RosNodeParams &params)
      : RosTopicSubNode<sensor_msgs::msg::Image>(name, conf, params)
  {
  }

  static PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<std::string>("topic_name"),
                              OutputPort<int>("climb_steps")});
  }

  NodeStatus onTick(const std::shared_ptr<sensor_msgs::msg::Image>& msg) override
  {
    
    // int valid_depth_count = 0;

    if (count < 20)
    {
      if (!msg)
      {
        RCLCPP_ERROR(node_->get_logger(), "Received null pointer for message.");
        // count--;
        return NodeStatus::SUCCESS;
      }

      float* depths = (float*)(&msg->data[0]);
      int u = msg->width / 2;
      int v = msg->height / 2;
      int centerIdx = u + msg->width * v;

      if (std::isnan(depths[centerIdx]))
      {
        RCLCPP_WARN(node_->get_logger(), "NaN value detected in depth data. Ignoring...");
        // count--;
        // continue;
        return NodeStatus::SUCCESS;
      }

      else
      {
      RCLCPP_INFO(node_->get_logger(), "Center distance : %g m", depths[centerIdx]);
      RCLCPP_INFO(node_->get_logger(), "Counts : %d ", count);
      new_val = new_val + depths[centerIdx];
      count++;
      // valid_depth_count++;
      // RCLCPP_INFO(node_->get_logger(), "Valid Depth Count = %d", valid_depth_count);
      // Simulate some delay
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      return NodeStatus::SUCCESS;
      }
      
    }
    
    if (count >= 20)
      {
        float avrg_val = new_val / 20;
        int climb_Steps = floor(avrg_val / 0.85); //0.8m is the length of each extension for now!!
        RCLCPP_INFO(node_->get_logger(), "Average center distance : %g m", avrg_val);
        RCLCPP_INFO(node_->get_logger(), "NUMBER OF CYCLES FOR CLIMBING : %d", climb_Steps);
        // n = climb_Steps;
        int nx = climb_Steps * 5;
        int ny = climb_Steps * 6;
        save_value_to_file(nx, "NumofStepsUP.txt");
        save_value_to_file(ny, "NumofStepsDOWN.txt");
        count = 0;  // Reset count for next execution
        new_val = 0;  // Reset new_val for next execution
        setOutput("climb_steps", climb_Steps);
        // sensor_msgs::msg::Image->unsubscribe();
        return NodeStatus::FAILURE;
      }
  
     // This should never be reached, but in case of some unexpected scenario
    RCLCPP_ERROR(node_->get_logger(), "Unexpected state reached in onTick()");
    return NodeStatus::SUCCESS;
  }
};


class CheckPLS : public SyncActionNode {
public:
    CheckPLS(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config) {}

    static PortsList providedPorts()
    {
        return { OutputPort<int>("home_climbsteps")};
    }

    NodeStatus tick() override
    {
        // Read the paths of the two text files from input ports
        // const std::string file1_path = getInput<std::string>("file1_path");
        // const std::string file2_path = getInput<std::string>("file2_path");

        // Open the first text file
        std::ifstream file1("NumofStepsUP.txt");
        if (!file1.is_open()) {
            std::cerr << "Failed to open file: NumofStepsUP.txt" << std::endl;
            return NodeStatus::FAILURE;
        }

        // Open the second text file
        std::ifstream file2("NumofStepsDOWN.txt");
        if (!file2.is_open()) {
            std::cerr << "Failed to open file: NumofStepsDOWN.txt" << std::endl;
            file1.close();
            return NodeStatus::FAILURE;
        }

        // Read the contents of the first file
        int content1;
        file1 >> content1;

        // Read the contents of the second file
        int content2;
        file2 >> content2;

        // Close the files
        file1.close();
        file2.close();
        
        // int climb_Steps = floor(avrg_val / 0.85); 

        // Compare the contents of the files
        if (content1 == 0 && content2 == 0) {
            std::cout << "Number of Cycles is zero (SUCCESS)!" << std::endl;
            setOutput("home_climbsteps", 0);
            return NodeStatus::SUCCESS;
        } else {
            std::cout << "Number of Cycles is NOT ZERO (FAILURE)!" << std::endl;

            if (content1 == 0)
            {
              if (content2 != 0)
              {
                int smth1 = floor(content2 / 6);
                int smth2 = content1 % 6;
                int smth3 = smth1 + smth2 - 1; // minus 1 because by default monkey will already be retracted by 1 time already from BT
  
                setOutput("home_climbsteps", smth3);
              }
              
            }

            else
            {
              int smth1 = (floor(content1 / 5) * 6);
              int smth2 = content1 % 5;
              int smth3 = smth1 + smth2 - 1;

              setOutput("home_climbsteps", smth3);
            }

            // setOutput("climb_steps", climb_Steps);
            return NodeStatus::FAILURE;
        }
    }
};



static const char* xml_text = R"(
<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4">
  <BehaviorTree ID="New0HDJIC">
    <Fallback>
      <Repeat num_cycles="3">
        <Sequence>
          <SubTree ID="OpenUC&amp;LC"/>
          <Fallback>
            <DMKEGetPos node_id="3"
                        next_pos="0"/>
            <DMKESetPosition node_id="3"
                             target_position="0"/>
          </Fallback>
          <Fallback>
            <DMKEGetPos node_id="2"
                        next_pos="0"/>
            <DMKESetPosition node_id="2"
                             target_position="0"/>
          </Fallback>
        </Sequence>
      </Repeat>
      <AlwaysSuccess name="AskForHelp"/>
    </Fallback>
  </BehaviorTree>

  <BehaviorTree ID="OpenUC&amp;LC">
    <Fallback>
      <Sequence>
        <Fallback>
          <DMKEGetPos node_id="2"
                      next_pos="550000"/>
          <DMKESetPosition node_id="2"
                           target_position="550000"/>
        </Fallback>
        <Fallback>
          <DMKEGetPos node_id="3"
                      next_pos="550000"/>
          <DMKESetPosition node_id="3"
                           target_position="550000"/>
        </Fallback>
      </Sequence>
      <AlwaysSuccess name="AskForHelp"/>
    </Fallback>
  </BehaviorTree>

  <!-- Description of Node Models (used by Groot) -->
  <TreeNodesModel>
    <Action ID="DMKEGetPos"
            editable="true">
      <input_port name="node_id"/>
      <input_port name="service_name"
                  default="/get_position"/>
      <input_port name="next_pos"/>
    </Action>
    <Action ID="DMKESetPosition"
            editable="true">
      <input_port name="node_id"/>
      <input_port name="action_name"
                  default="/set_position"/>
      <input_port name="target_position"/>
    </Action>
  </TreeNodesModel>

</root>
)";



// <root BTCPP_format="4">
//   <BehaviorTree ID="ROSwithBT_test1">
//     <Repeat num_cycles="2">
//       <Sequence>
//         <Delay delay_msec="3000">
//           <Fallback>
//             <GetPos service_name="/get_position" target_pos="500000"/>
//             <DMKESetPosition action_name="/set_position" target_position="500000"/>
//           </Fallback>
//         </Delay>
//         <Delay delay_msec="3000">
//           <Fallback>
//             <GetPos service_name="/get_position" target_pos="0"/>
//             <DMKESetPosition action_name="/set_position" target_position="0"/>
//           </Fallback>
//         </Delay>
//       </Sequence>
//     </Repeat>
//   </BehaviorTree>

//   <!-- Description of Node Models (used by Groot) -->
//   <TreeNodesModel/>

// </root>




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

  auto bt_node = std::make_shared<rclcpp::Node>("MonkeyClient");
  // provide the ROS node and the name of the action service
  RosNodeParams setpos_params; 
  setpos_params.nh = bt_node;
  // setpos_params.default_port_value = "set_position_server";

  factory.registerNodeType<DMKESetPosition>("DMKESetPosition", setpos_params);
  factory.registerNodeType<DMKEGetPosition>("GetPos", setpos_params);
  factory.registerNodeType<CylinderSetExtension>("CylinderSetExt", setpos_params);
  factory.registerNodeType<CylinderGetExtension>("GetExt", setpos_params);
  factory.registerNodeType<HeightDetection>("HeightDetection", setpos_params);
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

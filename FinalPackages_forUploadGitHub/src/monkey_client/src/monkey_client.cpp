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
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <chrono>
#include <math.h>
#include <fstream>
#include <stdexcept>


using namespace BT;
using namespace std::chrono;
// let's define these, for brevity
// using SetPosition = monkey_interface::action::SetPosition;
// using GetPosition = monkey_interface::srv::GetPosition;
// using GoalHandleSetPosition = rclcpp_action::ServerGoalHandle<SetPosition>;

// PUT ALL THE CLIENT IN THIS FILE SO THAT BT CAN TICK WHEN CALLED.
// - Service Client (dmke, cylinder)
// - Action client (dmke, cylinder)
// - Height Detection (zed)
// - CheckPLS
// - AskForHelp 

int count = 0;
float new_val = 0;

int DMKEopen = 550000;
int DMKEclose = 10000;
float LAextend = 1.1;
float LAretract = 0.0;


void save_value_to_file(int number, const std::string& file_path) {
    std::ofstream file(file_path);
    if (file.is_open()) {
        file << number;
        file.close();
    } else {
        throw std::runtime_error("Unable to open file for writing.");
    }
    // std::ofstream outfile(file_path); // Open file for writing

    // if (outfile.is_open()) {
    //     // Write to the file
    //     outfile << number;
    //     // outfile << "This is another line of text.\n";

    //     // Close the file
    //     outfile.close();
    //     std::cout << "File written successfully.\n";
    // } else {
    //     std::cout << "Unable to open file.\n";
    // }
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

    // std::ifstream infile(file_path); // Open file for reading
    // int value;
    // if (infile.is_open()) {
        
    //     infile >> value;
    //     // // Read integers until the end of the file
    //     // while (infile >> value) {
    //     //     std::cout << value << "\n";
    //     // }

    //     // Close the file
    //     infile.close();
        
    // } else {
    //     std::cout << "Unable to open file.\n";
    // }
    // return value;
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

    if (!getInput("node_id", goal.node_id)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get node_id from input port");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Using value for 'node_id': %d", goal.node_id);
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
    NodeStatus onResponseReceived(const Response::SharedPtr& response) override
  {
    // Log
    int position_got = response->positiongot;
    std::cout << getInput<int>("target_pos").value();
    
    if (position_got > getInput<int>("target_pos").value() + 5000 || position_got < getInput<int>("target_pos").value() - 5000) 
         {
         std::stringstream ss;
         ss << this->name() << " -> Position NOT within bounds (NODESTATUS:FAILURE): " << position_got;
         RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

         // setOutput("position", response->position);
         return NodeStatus::FAILURE;
         }


    else{
      std::stringstream ss;
      ss << this->name() << " -> Position within bounds (NODESTATUS:SUCCESS): " << position_got;
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
                              OutputPort<int>("climb_cycles")});
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
        int climb_cycles = floor(avrg_val / 0.8); //0.8m is the length of each extension for now!!
        RCLCPP_INFO(node_->get_logger(), "Average center distance : %g m", avrg_val);
        RCLCPP_INFO(node_->get_logger(), "NUMBER OF CYCLES FOR CLIMBING : %d", climb_cycles);

        // if (climb_cycles == 0)
        
        save_value_to_file(climb_cycles, "NumofCyclesUP.txt");
        save_value_to_file(climb_cycles, "NumofCyclesDOWN.txt");
        count = 0;  // Reset count for next execution
        new_val = 0;  // Reset new_val for next execution
        setOutput("climb_cycles", climb_cycles);
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
        return { OutputPort<int>("home_climbcycles")};
    }

    NodeStatus tick() override
    {
        // Read the paths of the two text files from input ports
        // const std::string file1_path = getInput<std::string>("file1_path");
        // const std::string file2_path = getInput<std::string>("file2_path");

        // Open the first text file
        std::ifstream file1("NumofCyclesUP.txt");
        if (!file1.is_open()) {
            std::cerr << "Failed to open file: NumofCyclesUP.txt" << std::endl;
            return NodeStatus::FAILURE;
        }

        // Open the second text file
        std::ifstream file2("NumofCyclesDOWN.txt");
        if (!file2.is_open()) {
            std::cerr << "Failed to open file: NumofCyclesDOWN.txt" << std::endl;
            file1.close();
            return NodeStatus::FAILURE;
        }

        // Read the contents of the first file
        int NoC_UP;
        file1 >> NoC_UP;

        // Read the contents of the second file
        int NoC_DOWN;
        file2 >> NoC_DOWN;

        // Close the files
        file1.close();
        file2.close();
        
        // int climb_Steps = floor(avrg_val / 0.85); 

        // Compare the contents of the files
        if (NoC_UP == 0 && NoC_DOWN == 0) {
            std::cout << "Number of Cycles is zero, Homing not required (SUCCESS)!" << std::endl;
            setOutput("home_climbcycles", 0);
            return NodeStatus::SUCCESS;
        } else {
            std::cout << "Number of Cycles is NOT ZERO, HOMING REQUIRED (FAILURE)!" << std::endl;

            if (NoC_UP == 0)
            {
              if (NoC_DOWN != 0)
              {
                int home_cycles = NoC_DOWN - 1;
                setOutput("home_climbcycles", home_cycles);
              }
            }

            else
            {
              int home_cycles = NoC_UP - 1;
              setOutput("home_climbcycles", home_cycles);
            }

            return NodeStatus::FAILURE;
        }
    }
};


class AskForHelp : public AlwaysFailureNode {
public:
    AskForHelp(const std::string& name)
        : AlwaysFailureNode(name) {}

    // static PortsList providedPorts()
    // {
    //     // return { OutputPort<int>("home_climbsteps")};
    // }

    BT::NodeStatus tick() override
    {
        rclcpp::shutdown();
        
        return NodeStatus::SUCCESS;
    }
    
};


class UpSubtract : public SyncActionNode {
public:
    UpSubtract(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config) {}

    static PortsList providedPorts()
    {
        return { InputPort<int>("UP_subtract")};
    }

    BT::NodeStatus tick() override
    {
      int x = read_value_from_file("NumofCyclesUP.txt");
      x += getInput<int>("UP_subtract").value();
      save_value_to_file(x, "NumofCyclesUP.txt");
      
      return NodeStatus::SUCCESS;
    }
    
};



class DownSubtract : public SyncActionNode {
public:
    DownSubtract(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config) {}

    static PortsList providedPorts()
    {
        return { InputPort<int>("DOWN_subtract")};
    }

    BT::NodeStatus tick() override
    {
      int y = read_value_from_file("NumofCyclesDOWN.txt");
      y += getInput<int>("DOWN_subtract").value();
      save_value_to_file(y, "NumofCyclesDOWN.txt");
      
      return NodeStatus::SUCCESS;
    }
    
};


// class ClimbingSystemBlackboard : public SyncActionNode{
// public:
//   ClimbingSystemBlackboard(const std::string& name, const NodeConfig& config)
//     : SyncActionNode(name, config)
//   {}

//   static PortsList providedPorts()
//   {
//     return {OutputPort<int>("DMKE_open"),
//             OutputPort<int>("DMKE_close")
//             // OutputPort<float>("LA_extend"),
//             // OutputPort<float>("LA_retract")
//             };
//   }

//   BT::NodeStatus tick() override
//   {
//     if (DMKEopen > 650000 || DMKEclose < -20000 || LAextend > 1.3 || LAretract < 0.0)
//     {
//       std::cout << "MOTOR/ACTUATOR VALUES EXCEEDED LIMIT (SETBLACKBOARD FAILURE) \n";
//       std::cout << "DMKE_open: " << DMKEopen << "\n";
//       std::cout << "DMKE_close: " << DMKEclose << "\n";
//       std::cout << "LA_extend: " << LAextend << "\n";
//       std::cout << "LA_retract: " << LAretract << "\n";
//       std::cout << "-----PLEASE REVISE THE VALUES-----" << std::endl;

//       return NodeStatus::FAILURE;
//     }

//     else 
//     {
//       setOutput("DMKE_open", DMKEopen);
//       setOutput("DMKE_close", DMKEclose);
//       // setOutput("LA_extend", LAextend);
//       // setOutput("LA_retract", LAretract);

//       std::cout << "MOTOR/ACTUATOR VALUES SUCCESSFULLY SET (SETBLACKBOARD SUCCESS)\n";
//       std::cout << "DMKE_open: " << DMKEopen << "\n";
//       std::cout << "DMKE_close: " << DMKEclose << "\n";
//       std::cout << "LA_extend: " << LAextend << "\n";
//       std::cout << "LA_retract: " << LAretract << std::endl;
    
//       return NodeStatus::SUCCESS;
//     }
//   }
// };

// static const char* xml_text = R"(
// <?xml version="1.0" encoding="UTF-8"?>
// <root BTCPP_format="4">
//   <BehaviorTree ID="GetPosTest">
//     <Repeat num_cycles="2">
//       <Sequence>
//         <DMKEGetPos node="2" service_name="/get_position" target_pos="0"/>
//         <DMKEGetPos node="2" service_name="/get_position" target_pos="500000"/>
//       </Sequence>
//     </Repeat>
//   </BehaviorTree>

//   <!-- Description of Node Models (used by Groot) -->
//   <TreeNodesModel>
//     <Action ID="DMKEGetPos"
//             editable="true">
//       <input_port name="node"/>
//       <input_port name="service_name"
//                   default="/get_position"/>
//       <input_port name="target_pos"/>
//     </Action>
//   </TreeNodesModel>

// </root>
// )";


// static const char* xml_text = R"(
// <?xml version="1.0" encoding="UTF-8"?>
// <root BTCPP_format="4">
//   <BehaviorTree ID="ClimbDown">
//     <Fallback>
//       <Sequence>
//         <RetryUntilSuccessful num_attempts="3">
//           <Fallback>
//             <DMKEGetPos node_id="3"
//                         service_name="/get_position"
//                         target_pos="{dmke_open}"/>
//             <DMKESetPosition node_id="3"
//                              action_name="/set_position"
//                              target_position="{dmke_open}"/>
//           </Fallback>
//         </RetryUntilSuccessful>
//         <RetryUntilSuccessful num_attempts="3">
//           <Fallback>
//             <DMKEGetPos node_id="3"
//                         service_name="/get_position"
//                         target_pos="{dmke_close}"/>
//             <DMKESetPosition node_id="3"
//                              action_name="/set_position"
//                              target_position="{dmke_close}"/>
//           </Fallback>
//         </RetryUntilSuccessful>
//         <RetryUntilSuccessful num_attempts="3">
//           <Fallback>
//             <DMKEGetPos node_id="2"
//                         service_name="/get_position"
//                         target_pos="{dmke_open}"/>
//             <DMKESetPosition node_id="2"
//                              action_name="/set_position"
//                              target_position="{dmke_open}"/>
//           </Fallback>
//         </RetryUntilSuccessful>
//         <RetryUntilSuccessful num_attempts="3">
//           <Fallback>
//             <DMKEGetPos node_id="2"
//                         service_name="/get_position"
//                         target_pos="{dmke_close}"/>
//             <DMKESetPosition node_id="2"
//                              action_name="/set_position"
//                              target_position="{dmke_close}"/>
//           </Fallback>
//         </RetryUntilSuccessful>
//       </Sequence>
//       <AlwaysSuccess name="AskForHelp"/>
//     </Fallback>
//   </BehaviorTree>

//   <BehaviorTree ID="ClimbUp">
//     <Fallback>
//       <Sequence>
//         <RetryUntilSuccessful num_attempts="3">
//           <Fallback>
//             <DMKEGetPos node_id="2"
//                         service_name="/get_position"
//                         target_pos="{dmke_open}"/>
//             <DMKESetPosition node_id="2"
//                              action_name="/set_position"
//                              target_position="{dmke_open}"/>
//           </Fallback>
//         </RetryUntilSuccessful>
//         <RetryUntilSuccessful num_attempts="3">
//           <Fallback>
//             <DMKEGetPos node_id="2"
//                         service_name="/get_position"
//                         target_pos="{dmke_close}"/>
//             <DMKESetPosition node_id="2"
//                              action_name="/set_position"
//                              target_position="{dmke_close}"/>
//           </Fallback>
//         </RetryUntilSuccessful>
//         <RetryUntilSuccessful num_attempts="3">
//           <Fallback>
//             <DMKEGetPos node_id="3"
//                         service_name="/get_position"
//                         target_pos="{dmke_open}"/>
//             <DMKESetPosition node_id="3"
//                              action_name="/set_position"
//                              target_position="{dmke_open}"/>
//           </Fallback>
//         </RetryUntilSuccessful>
//         <RetryUntilSuccessful num_attempts="3">
//           <Fallback>
//             <DMKEGetPos node_id="3"
//                         service_name="/get_position"
//                         target_pos="{dmke_close}"/>
//             <DMKESetPosition node_id="3"
//                              action_name="/set_position"
//                              target_position="{dmke_close}"/>
//           </Fallback>
//         </RetryUntilSuccessful>
//       </Sequence>
//       <AlwaysSuccess name="AskForHelp"/>
//     </Fallback>
//   </BehaviorTree>

//   <BehaviorTree ID="DoubleClamp">
//     <Fallback>
//       <Sequence>
//         <Script code="dmke_open:=550000; dmke_close:=10000"/>
//         <SubTree ID="StartingPosition"
//                  _autoremap="true"/>
//         <Repeat num_cycles="3">
//           <SubTree ID="ClimbUp"
//                    _autoremap="true"/>
//         </Repeat>
//         <Delay delay_msec="10000">
//           <Repeat num_cycles="3">
//             <SubTree ID="ClimbDown"
//                      _autoremap="true"/>
//           </Repeat>
//         </Delay>
//         <SubTree ID="EndingPosition"
//                  _autoremap="true"/>
//       </Sequence>
//       <AlwaysSuccess name="AskForHelp"/>
//     </Fallback>
//   </BehaviorTree>

//   <BehaviorTree ID="EndingPosition">
//     <Fallback>
//       <Sequence>
//         <RetryUntilSuccessful num_attempts="3">
//           <Fallback>
//             <DMKEGetPos node_id="2"
//                         service_name="/get_position"
//                         target_pos="{dmke_open}"/>
//             <DMKESetPosition node_id="2"
//                              action_name="/set_position"
//                              target_position="{dmke_open}"/>
//           </Fallback>
//         </RetryUntilSuccessful>
//         <RetryUntilSuccessful num_attempts="3">
//           <Fallback>
//             <DMKEGetPos node_id="3"
//                         service_name="/get_position"
//                         target_pos="{dmke_open}"/>
//             <DMKESetPosition node_id="3"
//                              action_name="/set_position"
//                              target_position="{dmke_open}"/>
//           </Fallback>
//         </RetryUntilSuccessful>
//       </Sequence>
//       <AlwaysSuccess name="AskForHelp"/>
//     </Fallback>
//   </BehaviorTree>

//   <BehaviorTree ID="StartingPosition">
//     <Fallback>
//       <Sequence>
//         <RetryUntilSuccessful num_attempts="3">
//           <Fallback>
//             <DMKEGetPos node_id="2"
//                         service_name="/get_position"
//                         target_pos="{dmke_open}"/>
//             <DMKESetPosition node_id="2"
//                              action_name="/set_position"
//                              target_position="{dmke_open}"/>
//           </Fallback>
//         </RetryUntilSuccessful>
//         <RetryUntilSuccessful num_attempts="3">
//           <Fallback>
//             <DMKEGetPos node_id="3"
//                         service_name="/get_position"
//                         target_pos="{dmke_close}"/>
//             <DMKESetPosition node_id="3"
//                              action_name="/set_position"
//                              target_position="{dmke_close}"/>
//           </Fallback>
//         </RetryUntilSuccessful>
//       </Sequence>
//       <AlwaysSuccess name="AskForHelp"/>
//     </Fallback>
//   </BehaviorTree>

//   <!-- Description of Node Models (used by Groot) -->
//   <TreeNodesModel>
//     <Action ID="DMKEGetPos"
//             editable="true">
//       <input_port name="node_id"/>
//       <input_port name="service_name"
//                   default="/get_position"/>
//       <input_port name="target_pos"/>
//     </Action>
//     <Action ID="DMKESetPosition"
//             editable="true">
//       <input_port name="node_id"/>
//       <input_port name="action_name"
//                   default="/set_position"/>
//       <input_port name="target_position"/>
//     </Action>
//   </TreeNodesModel>

// </root>
// )";

// static const char* xml_text = R"(
// <?xml version="1.0" encoding="UTF-8"?>
// <root BTCPP_format="4">
//   <BehaviorTree ID="ClimbDown">
//     <Fallback>
//       <Sequence>
//         <RetryUntilSuccessful num_attempts="3">
//           <Fallback>
//             <LAGetExt service_name="/GetExtension"
//                       target_ext="{la_extend}"/>
//             <LASetExtension action_name="/SetExtension"
//                             target_extension="{la_extend}"/>
//           </Fallback>
//         </RetryUntilSuccessful>
//         <RetryUntilSuccessful num_attempts="3">
//           <Fallback>
//             <LAGetExt service_name="/GetExtension"
//                       target_ext="{la_retract}"/>
//             <LASetExtension action_name="/SetExtension"
//                             target_extension="{la_retract}"/>
//           </Fallback>
//         </RetryUntilSuccessful>
//         <DownSubtract DOWN_subtract="-1"/>
//       </Sequence>
//       <AlwaysSuccess name="AskForHelp"/>
//     </Fallback>
//   </BehaviorTree>

//   <BehaviorTree ID="ClimbUp">
//     <Fallback>
//       <Sequence>
//         <RetryUntilSuccessful num_attempts="3">
//           <Fallback>
//             <LAGetExt service_name="/GetExtension"
//                       target_ext="{la_extend}"/>
//             <LASetExtension action_name="/SetExtension"
//                             target_extension="{la_extend}"/>
//           </Fallback>
//         </RetryUntilSuccessful>
//         <RetryUntilSuccessful num_attempts="3">
//           <Fallback>
//             <LAGetExt service_name="/GetExtension"
//                       target_ext="{la_retract}"/>
//             <LASetExtension action_name="/SetExtension"
//                             target_extension="{la_retract}"/>
//           </Fallback>
//         </RetryUntilSuccessful>
//         <UpSubtract UP_subtract="-1"/>
//       </Sequence>
//       <AlwaysSuccess name="AskForHelp"/>
//     </Fallback>
//   </BehaviorTree>

//   <BehaviorTree ID="EndingPosition">
//     <Fallback>
//       <Sequence>
//         <RetryUntilSuccessful num_attempts="3">
//           <Fallback>
//             <LAGetExt service_name="/GetExtension"
//                       target_ext="0.0"/>
//             <LASetExtension action_name="/SetExtension"
//                             target_extension="0.0"/>
//           </Fallback>
//         </RetryUntilSuccessful>
//       </Sequence>
//       <AlwaysSuccess name="AskForHelp"/>
//     </Fallback>
//   </BehaviorTree>

//   <BehaviorTree ID="LAHD">
//     <Fallback>
//       <Sequence>
//         <Script code="dmke_open:=550000; dmke_close:=10000; la_extend:=1.0; la_retract:=0.1"/>
//         <SubTree ID="PowerLossScenario"
//                  _autoremap="true"/>
//         <Inverter>
//           <KeepRunningUntilFailure>
//             <HeightDetection topic_name="/zed/zed_node/depth/depth_registered"
//                              climb_cycles="{climb_cycles}"/>
//           </KeepRunningUntilFailure>
//         </Inverter>
//         <SubTree ID="StartingPosition"
//                  _autoremap="true"/>
//         <Repeat num_cycles="{climb_cycles}">
//           <SubTree ID="ClimbUp"
//                    _autoremap="true"/>
//         </Repeat>
//         <Delay delay_msec="10000">
//           <Repeat num_cycles="{climb_cycles}">
//             <SubTree ID="ClimbDown"
//                      _autoremap="true"/>
//           </Repeat>
//         </Delay>
//         <SubTree ID="EndingPosition"
//                  _autoremap="true"/>
//       </Sequence>
//       <AlwaysSuccess name="AskForHelp"/>
//     </Fallback>
//   </BehaviorTree>

//   <BehaviorTree ID="PowerLossScenario">
//     <Fallback>
//       <CheckPLS home_climbcycles="{home_climbcycles}"/>
//       <Sequence>
//         <RetryUntilSuccessful num_attempts="3">
//           <Fallback>
//             <LAGetExt service_name="/GetExtension"
//                       target_ext="{la_retract}"/>
//             <LASetExtension action_name="/SetExtension"
//                             target_extension="{la_retract}"/>
//           </Fallback>
//         </RetryUntilSuccessful>
//         <SubTree ID="EndingPosition"
//                  _autoremap="true"/>
//         <AlwaysSuccess name="AskForHelp"/>
//       </Sequence>
//     </Fallback>
//   </BehaviorTree>

//   <BehaviorTree ID="StartingPosition">
//     <Fallback>
//       <Sequence>
//         <RetryUntilSuccessful num_attempts="3">
//           <Fallback>
//             <LAGetExt service_name="/GetExtension"
//                       target_ext="{la_retract}"/>
//             <LASetExtension action_name="/SetExtension"
//                             target_extension="{la_retract}"/>
//           </Fallback>
//         </RetryUntilSuccessful>
//       </Sequence>
//       <AlwaysSuccess name="AskForHelp"/>
//     </Fallback>
//   </BehaviorTree>

//   <!-- Description of Node Models (used by Groot) -->
//   <TreeNodesModel>
//     <Action ID="CheckPLS"
//             editable="true">
//       <output_port name="home_climbcycles"/>
//     </Action>
//     <Action ID="DownSubtract"
//             editable="true">
//       <input_port name="DOWN_subtract"
//                   default="-1"/>
//     </Action>
//     <Condition ID="HeightDetection"
//                editable="true">
//       <input_port name="topic_name"/>
//       <output_port name="climb_cycles"/>
//     </Condition>
//     <Action ID="LAGetExt"
//             editable="true">
//       <input_port name="service_name"
//                   default="/GetExtension"/>
//       <input_port name="target_ext"/>
//     </Action>
//     <Action ID="LASetExtension"
//             editable="true">
//       <input_port name="action_name"
//                   default="/SetExtension"/>
//       <input_port name="target_extension"/>
//     </Action>
//     <Action ID="UpSubtract"
//             editable="true">
//       <input_port name="UP_subtract"
//                   default="-1"/>
//     </Action>
//   </TreeNodesModel>

// </root>
// )";

static const char* xml_text = R"(
<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4">
  <BehaviorTree ID="ClimbDown">
    <Fallback>
      <Sequence>
        <RetryUntilSuccessful num_attempts="3">
          <Fallback>
            <DMKEGetPos node_id="3"
                        service_name="/get_position"
                        target_pos="{dmke_open}"/>
            <DMKESetPosition node_id="3"
                             action_name="/set_position"
                             target_position="{dmke_open}"/>
          </Fallback>
        </RetryUntilSuccessful>
        <RetryUntilSuccessful num_attempts="3">
          <Fallback>
            <DMKEGetPos node_id="3"
                        service_name="/get_position"
                        target_pos="{dmke_close}"/>
            <DMKESetPosition node_id="3"
                             action_name="/set_position"
                             target_position="{dmke_close}"/>
          </Fallback>
        </RetryUntilSuccessful>
        <RetryUntilSuccessful num_attempts="3">
          <Fallback>
            <DMKEGetPos node_id="2"
                        service_name="/get_position"
                        target_pos="{dmke_open}"/>
            <DMKESetPosition node_id="2"
                             action_name="/set_position"
                             target_position="{dmke_open}"/>
          </Fallback>
        </RetryUntilSuccessful>
        <RetryUntilSuccessful num_attempts="3">
          <Fallback>
            <DMKEGetPos node_id="2"
                        service_name="/get_position"
                        target_pos="{dmke_close}"/>
            <DMKESetPosition node_id="2"
                             action_name="/set_position"
                             target_position="{dmke_close}"/>
          </Fallback>
        </RetryUntilSuccessful>
        <DownSubtract DOWN_subtract="-1"/>
      </Sequence>
      <AlwaysSuccess name="AskForHelp"/>
    </Fallback>
  </BehaviorTree>

  <BehaviorTree ID="ClimbUp">
    <Fallback>
      <Sequence>
        <RetryUntilSuccessful num_attempts="3">
          <Fallback>
            <DMKEGetPos node_id="2"
                        service_name="/get_position"
                        target_pos="{dmke_open}"/>
            <DMKESetPosition node_id="2"
                             action_name="/set_position"
                             target_position="{dmke_open}"/>
          </Fallback>
        </RetryUntilSuccessful>
        <RetryUntilSuccessful num_attempts="3">
          <Fallback>
            <DMKEGetPos node_id="2"
                        service_name="/get_position"
                        target_pos="{dmke_close}"/>
            <DMKESetPosition node_id="2"
                             action_name="/set_position"
                             target_position="{dmke_close}"/>
          </Fallback>
        </RetryUntilSuccessful>
        <RetryUntilSuccessful num_attempts="3">
          <Fallback>
            <DMKEGetPos node_id="3"
                        service_name="/get_position"
                        target_pos="{dmke_open}"/>
            <DMKESetPosition node_id="3"
                             action_name="/set_position"
                             target_position="{dmke_open}"/>
          </Fallback>
        </RetryUntilSuccessful>
        <RetryUntilSuccessful num_attempts="3">
          <Fallback>
            <DMKEGetPos node_id="3"
                        service_name="/get_position"
                        target_pos="{dmke_close}"/>
            <DMKESetPosition node_id="3"
                             action_name="/set_position"
                             target_position="{dmke_close}"/>
          </Fallback>
        </RetryUntilSuccessful>
        <UpSubtract UP_subtract="-1"/>
      </Sequence>
      <AlwaysSuccess name="AskForHelp"/>
    </Fallback>
  </BehaviorTree>

  <BehaviorTree ID="DoubleClampHD">
    <Fallback>
      <Sequence>
        <Script code="dmke_open:=550000; dmke_close:=10000"/>
        <SubTree ID="PowerLossScenario"
                 _autoremap="true"/>
        <Inverter>
          <KeepRunningUntilFailure>
            <HeightDetection topic_name="/zed/zed_node/depth/depth_registered"
                             climb_cycles="{climb_cycles}"/>
          </KeepRunningUntilFailure>
        </Inverter>
        <SubTree ID="StartingPosition"
                 _autoremap="true"/>
        <Repeat num_cycles="{climb_cycles}">
          <SubTree ID="ClimbUp"
                   _autoremap="true"/>
        </Repeat>
        <Delay delay_msec="10000">
          <Repeat num_cycles="{climb_cycles}">
            <SubTree ID="ClimbDown"
                     _autoremap="true"/>
          </Repeat>
        </Delay>
        <SubTree ID="EndingPosition"
                 _autoremap="true"/>
      </Sequence>
      <AlwaysSuccess name="AskForHelp"/>
    </Fallback>
  </BehaviorTree>

  <BehaviorTree ID="EndingPosition">
    <Fallback>
      <Sequence>
        <RetryUntilSuccessful num_attempts="3">
          <Fallback>
            <DMKEGetPos node_id="2"
                        service_name="/get_position"
                        target_pos="{dmke_open}"/>
            <DMKESetPosition node_id="2"
                             action_name="/set_position"
                             target_position="{dmke_open}"/>
          </Fallback>
        </RetryUntilSuccessful>
        <RetryUntilSuccessful num_attempts="3">
          <Fallback>
            <DMKEGetPos node_id="3"
                        service_name="/get_position"
                        target_pos="{dmke_open}"/>
            <DMKESetPosition node_id="3"
                             action_name="/set_position"
                             target_position="{dmke_open}"/>
          </Fallback>
        </RetryUntilSuccessful>
      </Sequence>
      <AlwaysSuccess name="AskForHelp"/>
    </Fallback>
  </BehaviorTree>

  <BehaviorTree ID="PowerLossScenario">
    <Fallback>
      <CheckPLS home_climbcycles="{home_climbcycles}"/>
      <Sequence>
        <RetryUntilSuccessful num_attempts="3">
          <Fallback>
            <DMKEGetPos node_id="3"
                        service_name="/get_position"
                        target_pos="{dmke_close}"/>
            <DMKESetPosition node_id="3"
                             action_name="/set_position"
                             target_position="{dmke_close}"/>
          </Fallback>
        </RetryUntilSuccessful>
        <RetryUntilSuccessful num_attempts="3">
          <Fallback>
            <DMKEGetPos node_id="2"
                        service_name="/get_position"
                        target_pos="{dmke_open}"/>
            <DMKESetPosition node_id="2"
                             action_name="/set_position"
                             target_position="{dmke_open}"/>
          </Fallback>
        </RetryUntilSuccessful>
        <RetryUntilSuccessful num_attempts="3">
          <Fallback>
            <DMKEGetPos node_id="2"
                        service_name="/get_position"
                        target_pos="{dmke_close}"/>
            <DMKESetPosition node_id="2"
                             action_name="/set_position"
                             target_position="{dmke_close}"/>
          </Fallback>
        </RetryUntilSuccessful>
        <Repeat num_cycles="{home_climbcycles}">
          <SubTree ID="ClimbDown"
                   _autoremap="true"/>
        </Repeat>
        <SubTree ID="EndingPosition"
                 _autoremap="true"/>
        <AlwaysSuccess name="AskForHelp"/>
      </Sequence>
    </Fallback>
  </BehaviorTree>

  <BehaviorTree ID="StartingPosition">
    <Fallback>
      <Sequence>
        <RetryUntilSuccessful num_attempts="3">
          <Fallback>
            <DMKEGetPos node_id="2"
                        service_name="/get_position"
                        target_pos="{dmke_open}"/>
            <DMKESetPosition node_id="2"
                             action_name="/set_position"
                             target_position="{dmke_open}"/>
          </Fallback>
        </RetryUntilSuccessful>
        <RetryUntilSuccessful num_attempts="3">
          <Fallback>
            <DMKEGetPos node_id="3"
                        service_name="/get_position"
                        target_pos="{dmke_close}"/>
            <DMKESetPosition node_id="3"
                             action_name="/set_position"
                             target_position="{dmke_close}"/>
          </Fallback>
        </RetryUntilSuccessful>
      </Sequence>
      <AlwaysSuccess name="AskForHelp"/>
    </Fallback>
  </BehaviorTree>

  <!-- Description of Node Models (used by Groot) -->
  <TreeNodesModel>
    <Action ID="CheckPLS"
            editable="true">
      <output_port name="home_climbcycles"/>
    </Action>
    <Action ID="DMKEGetPos"
            editable="true">
      <input_port name="node_id"/>
      <input_port name="service_name"
                  default="/get_position"/>
      <input_port name="target_pos"/>
    </Action>
    <Action ID="DMKESetPosition"
            editable="true">
      <input_port name="node_id"/>
      <input_port name="action_name"
                  default="/set_position"/>
      <input_port name="target_position"/>
    </Action>
    <Action ID="DownSubtract"
            editable="true">
      <input_port name="DOWN_subtract"
                  default="-1"/>
    </Action>
    <Condition ID="HeightDetection"
               editable="true">
      <input_port name="topic_name"/>
      <output_port name="climb_cycles"/>
    </Condition>
    <Action ID="UpSubtract"
            editable="true">
      <input_port name="UP_subtract"
                  default="-1"/>
    </Action>
  </TreeNodesModel>

</root>
)";

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
  factory.registerNodeType<DMKEGetPosition>("DMKEGetPos", setpos_params);
  factory.registerNodeType<CylinderSetExtension>("LASetExtension", setpos_params);
  factory.registerNodeType<CylinderGetExtension>("LAGetExt", setpos_params);
  factory.registerNodeType<HeightDetection>("HeightDetection", setpos_params);
  factory.registerNodeType<UpSubtract>("UpSubtract");
  factory.registerNodeType<DownSubtract>("DownSubtract");
  
  // factory.registerNodeType<ClimbingSystemBlackboard>("ClimbingSystemBlackboard");
  factory.registerNodeType<CheckPLS>("CheckPLS");
  factory.registerNodeType<AskForHelp>("AskForHelp");

  // Create the behavior tree using the XML description
  // auto tree = factory.createTreeFromText(xml_text);

  factory.registerBehaviorTreeFromText(xml_text);
  auto tree = factory.createTree("DoubleClampHD");

  // Run the behavior tree until it finishes
  tree.tickWhileRunning();
  // Spin
  rclcpp::spin(bt_node);
  rclcpp::shutdown();
  return 0;

}

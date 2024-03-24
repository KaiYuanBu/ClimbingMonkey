#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <chrono>

using namespace BT;
using namespace std::chrono;

int count = 0;
float new_val = 0;
int execution_count = 0;

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
                              OutputPort<float>("avg_height")});
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
        RCLCPP_INFO(node_->get_logger(), "Average center distance : %g m", avrg_val);
        count = 0;  // Reset count for next execution
        new_val = 0;  // Reset new_val for next execution
        setOutput("avg_height", avrg_val);
        // sensor_msgs::msg::Image->unsubscribe();
        return NodeStatus::FAILURE;
        // rclcpp::shutdown();
        // 
        
        
      }
    // if (valid_depth_count > 0)
    // {
    //   float avrg_val = new_val / valid_depth_count;
    //   RCLCPP_INFO(node_->get_logger(), "Average center distance of %d valid data : %g m", valid_depth_count, avrg_val);
    //   // Perform further processing with avrg_val if needed
    // }
    // else
    // {
    //   RCLCPP_WARN(node_->get_logger(), "No valid depth data received.");
    // }

    // return NodeStatus::FAILURE;

     // This should never be reached, but in case of some unexpected scenario
    RCLCPP_ERROR(node_->get_logger(), "Unexpected state reached in onTick()");
    return NodeStatus::SUCCESS;
  }
};

// static const char *xml_text = R"(
// <root BTCPP_format="4">
//   <BehaviorTree ID="zed2i_test">
//     <Repeat num_cycles="20">
//       <Fallback>
//         <HeightDetection topic_name="/zed/zed_node/depth/depth_registered"/>
//         <Delay delay_msec="5000">
//           <HeightDetection topic_name="/zed/zed_node/depth/depth_registered"/>
//         </Delay>
//       </Fallback>
//     </Repeat>
//   </BehaviorTree>
// </root>
// )";

// static const char *xml_text = R"(
// <root BTCPP_format="4">
//   <BehaviorTree ID="zed2i_test">
//     <Repeat num_cycles="21">
//       <HeightDetection topic_name="/zed/zed_node/depth/depth_registered"/>
//     </Repeat>
//   </BehaviorTree>
// </root>
// )";

static const char *xml_text = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="zed2i_test">
    <KeepRunningUntilFailure>
      <HeightDetection topic_name="/zed/zed_node/depth/depth_registered"/>
    </KeepRunningUntilFailure>
  </BehaviorTree>
</root>
)";

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("zed2i_bt_sub_test");

  BehaviorTreeFactory factory;

  RosNodeParams params;
  params.nh = nh;
  // params.default_port_value = "/zed/zed_node/depth/depth_registered";
  factory.registerNodeType<HeightDetection>("HeightDetection", params);

  auto tree = factory.createTreeFromText(xml_text);

  // rclcpp::spin(nh);

  // while (rclcpp::ok())
  // {
    // rclcpp::spin_some(nh);
    tree.tickWhileRunning();
  //   // break;
  //   // if (count >= 20)
  //   // {
  //   //   break;
  //   }
  // }
  rclcpp::spin(nh);
  // rclcpp::spin_some(nh);
  rclcpp::shutdown();
  // while (rclcpp::ok() && execution_count < 2)
  // {
  //   rclcpp::spin_some(nh);
  //   tree.tickWhileRunning();

  //   if (tree.rootNode()->status() == NodeStatus::IDLE)
  //   {
  //     execution_count++;
  //     RCLCPP_INFO(nh->get_logger(), "Behavior Tree Execution Count: %d", execution_count);
  //   }

  //   std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Add a small delay
  // }
  
  // rclcpp::shutdown();

  return 0;
}
















// #include "behaviortree_ros2/bt_topic_sub_node.hpp"
// #include "sensor_msgs/msg/image.hpp"
// #include <iostream>
// #include <rclcpp/rclcpp.hpp>
// #include <behaviortree_cpp/bt_factory.h>
// #include <chrono>

// using namespace BT;
// using namespace std::chrono;

// class HeightDetection : public RosTopicSubNode<sensor_msgs::msg::Image>
// {
// public:
//   HeightDetection(const std::string &name,
//                   const NodeConfig &conf,
//                   const RosNodeParams &params)
//       : RosTopicSubNode<sensor_msgs::msg::Image>(name, conf, params)
//   {
//   }

//   static PortsList providedPorts()
//   {
//     return providedBasicPorts({InputPort<std::string>("topic_name")});
//   }

//   // NodeStatus onTick(const std::shared_ptr<sensor_msgs::msg::Image>& msg) override
//   // {
//   //   float new_val = 0;

//   //   // for (int counter = 0; counter < 20; counter++)
//   //   // {
//   //     float *depths = (float *)(&msg->data[0]);

//   //     int u = msg->width / 2;
//   //     int v = msg->height / 2;

//   //     int centerIdx = u + msg->width * v;
//   //     float newcenterIdx = depths[centerIdx];

//   //     std::stringstream ss;
//   //     ss << this->name() << " -> Center distance : " << newcenterIdx << " m";
//   //     RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

//   //     new_val = new_val + newcenterIdx;
//   //     sleep(0.5);
//   //   // }

//   //   // float avrg_val = new_val / 20;
//   //   // setOutput("height_detected", avrg_val);
    
//   //   return NodeStatus::SUCCESS;
//   // }
// NodeStatus onTick(const std::shared_ptr<sensor_msgs::msg::Image>& msg) override
// {
//   float new_val = 0;
//   for (int counter=0; counter < 20; counter ++){
//   // if (!msg)
//   // {
//   //   RCLCPP_ERROR(node_->get_logger(), "Received null pointer for message.");
//   //   return NodeStatus::FAILURE;
//   // }

//   // if (msg->data.empty())
//   // {
//   //   RCLCPP_ERROR(node_->get_logger(), "Received empty data in the message.");
//   //   return NodeStatus::FAILURE;
//   // }
  
//   if (msg)
//   {
//   float* depths = (float*)(&msg->data[0]);
//   int u = msg->width / 2;
//   int v = msg->height / 2;
//   int centerIdx = u + msg->width * v;
//   // RCLCPP_INFO(node_->get_logger(), "Loop : %d ", counter);
//   RCLCPP_INFO(node_->get_logger(), "Center distance : %g m", depths[centerIdx]);

    
//   }
  

//   // else
//   // {
//   //   RCLCPP_ERROR(node_->get_logger(), "Invalid index for depths array.");
//   //   return NodeStatus::FAILURE;
//   // }

//   // Use new_val for further processing
//   if (counter == 20)
//   {
    
//   }
//   }

//   return NodeStatus::SUCCESS;
// }

// };

// static const char *xml_text = R"(
// <root BTCPP_format="4">
//   <BehaviorTree ID="zed2i_test">
//     <Sequence>
//       <HeightDetection topic_name="/zed/zed_node/depth/depth_registered"/>
//       <Delay delay_msec="3000">
//         <HeightDetection topic_name="/zed/zed_node/depth/depth_registered"/>
//       </Delay>
//     </Sequence>
//   </BehaviorTree>
// </root>
// )";

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   auto nh = std::make_shared<rclcpp::Node>("zed2i_bt_sub_test");

//   BehaviorTreeFactory factory;

//   RosNodeParams params;
//   params.nh = nh;
//   params.default_port_value = "/zed/zed_node/depth/depth_registered";
//   factory.registerNodeType<HeightDetection>("HeightDetection", params);

//   auto tree = factory.createTreeFromText(xml_text);

//   // rclcpp::executors::SingleThreadedExecutor executor;
//   // executor.add_node(nh);
//   // rclcpp::spin(nh);
  
//   // tree.tickOnce();
//     while (rclcpp::ok())
// {
//   rclcpp::spin_some(nh);
//   tree.tickWhileRunning();
//   std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Add a small delay
// }

//   rclcpp::shutdown();
 
//   // rclcpp::shutdown();
//   return 0;
// }
